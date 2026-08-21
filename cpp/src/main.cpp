// pico-sdk firmware for the Engr401 ELT3 "Cart and Rail" PID lab.
//
// C++ port of the Embassy/Rust firmware in `src/main.rs`, kept deliberately
// close to it line-for-line so the two can be compared directly. See
// `cpp/README.md` for a rundown of where the two necessarily diverge.
//
// Target board: Raspberry Pi Pico (RP2040).
//
// Wiring, per the lab handout:
//   GND         -> demonstrator GND
//   GPIO16      -> demonstrator Row 18   (servo/valve PWM control signal)
//   GPIO2       -> demonstrator Row 27   (ultrasonic sensor TRIG)
//   GPIO3       -> demonstrator Row 29   (ultrasonic sensor ECHO)
//   GPIO26/ADC0 -> setpoint potentiometer wiper
//
// The handout only documents the four demonstrator-breadboard wires
// (GND, GPIO16, GPIO2, GPIO3) since the setpoint potentiometer and the rest of
// the Pico wiring were presumably set up in an earlier lesson in the series.
// This firmware makes two concrete, clearly-labelled assumptions about
// hardware that isn't pinned down by the handout alone -- see the "Hardware
// assumptions" section of the top-level README.md before wiring up:
//   1. Actual cart position is sensed with an HC-SR04-style ultrasonic sensor
//      (TRIG on GPIO2, ECHO on GPIO3).
//   2. The controller command drives a standard 1-2ms hobby servo (valve or
//      flap) on GPIO16, with the 0.0..=1.0 command linearly mapped onto the
//      servo's pulse width.
// Both are easy to swap out for different hardware -- see the constants and
// `measure_distance_cm` below.

#include <cstdint>
#include <cstdio>
#include <optional>

#include "cart/pid.hpp"
#include "cart/rail.hpp"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

namespace {

// --- Pin assignments (see the wiring table above) ---
constexpr std::uint32_t PIN_TRIG = 2;
constexpr std::uint32_t PIN_ECHO = 3;
constexpr std::uint32_t PIN_SERVO = 16;
constexpr std::uint32_t PIN_POT = 26;
constexpr std::uint32_t ADC_INPUT_POT = 0;  // GPIO26 is ADC0

// Control loop rate. 20 Hz is comfortably faster than the cart's mechanical
// dynamics and leaves plenty of headroom for the ultrasonic sensor's ~30ms
// round-trip time.
constexpr std::uint32_t CONTROL_PERIOD_MS = 50;

// RP2040 ADC resolution (12-bit).
constexpr std::uint16_t ADC_MAX = 4095;

// Standard hobby-servo pulse-width range (1ms-2ms within a 20ms/50Hz frame).
constexpr std::uint16_t SERVO_PULSE_MIN_US = 1000;
constexpr std::uint16_t SERVO_PULSE_MAX_US = 2000;
constexpr std::uint16_t SERVO_FRAME_US = 20000;

// System clock is 125MHz by default; dividing by 125 gives a 1MHz PWM counter,
// i.e. 1 count = 1us, so `wrap`/`level` can be set directly in microseconds.
constexpr float PWM_CLOCK_DIVIDER = 125.0F;

// How long to wait for an ultrasonic echo before giving up on a sample. ~30ms
// corresponds to roughly a 5m round trip, well beyond the rail length.
constexpr std::uint32_t ULTRASONIC_TIMEOUT_US = 30000;

// Starting PID gains for Step 5 of the lab handout ("determine reasonable PID
// gains to begin your optimization process").
//
// The handout gives the expected operating ranges: error e(t) spans roughly
// +-30cm and the controller command needs to swing +-0.36 around its 0.54
// base. A proportional-only gain that can just reach the full command swing at
// the full expected error is a reasonable, physically-motivated starting point:
//     Kp ~= 0.36 / 30cm = 0.012
// Start tuning from here: increase Kp until the cart starts to oscillate
// around the setpoint, back off a bit, then add a small Kd to damp the
// oscillation, and finally a small Ki to remove any steady-state error.
constexpr cart::PidGains INITIAL_GAINS{0.012F, 0.0F, 0.0F};

// Anti-windup clamp on the accumulated integral term (cm*s). Prevents the
// integral from running away while Ki is still being tuned up from zero.
constexpr float INTEGRAL_LIMIT = 15.0F;

// Busy-wait until `pin` reads `level`, giving up after `timeout_us`. Returns
// false on timeout. `time_us_32()` wraps every ~71 minutes; unsigned
// subtraction makes the elapsed-time comparison correct across that wrap.
bool wait_for_level(std::uint32_t pin, bool level, std::uint32_t timeout_us) {
    const std::uint32_t start = time_us_32();
    while (gpio_get(pin) != level) {
        if (time_us_32() - start > timeout_us) {
            return false;
        }
    }
    return true;
}

// Trigger an HC-SR04-style ultrasonic sensor and time its echo pulse to get a
// distance in centimeters. Returns `std::nullopt` on timeout (no echo received
// -- e.g. the cart is out of range or the sensor isn't connected yet).
std::optional<float> measure_distance_cm() {
    gpio_put(PIN_TRIG, true);
    busy_wait_us(10);
    gpio_put(PIN_TRIG, false);

    if (!wait_for_level(PIN_ECHO, true, ULTRASONIC_TIMEOUT_US)) {
        return std::nullopt;
    }
    const std::uint32_t start = time_us_32();

    if (!wait_for_level(PIN_ECHO, false, ULTRASONIC_TIMEOUT_US)) {
        return std::nullopt;
    }
    const std::uint32_t echo_us = time_us_32() - start;

    return cart::distance_from_echo_micros(echo_us);
}

}  // namespace

int main() {
    stdio_init_all();

    // --- Setpoint potentiometer (desired cart position) ---
    adc_init();
    adc_gpio_init(PIN_POT);
    adc_select_input(ADC_INPUT_POT);

    // --- Ultrasonic range sensor (actual cart position) ---
    gpio_init(PIN_TRIG);
    gpio_set_dir(PIN_TRIG, GPIO_OUT);
    gpio_put(PIN_TRIG, false);
    gpio_init(PIN_ECHO);
    gpio_set_dir(PIN_ECHO, GPIO_IN);
    // The RP2040 pads come out of reset with a pull-down enabled; the echo line
    // is driven by the sensor, so disable it to match the Rust port's
    // `Input::new(p.PIN_3, Pull::None)`.
    gpio_disable_pulls(PIN_ECHO);

    // --- Servo/valve PWM output ---
    gpio_set_function(PIN_SERVO, GPIO_FUNC_PWM);
    const std::uint32_t servo_slice = pwm_gpio_to_slice_num(PIN_SERVO);
    pwm_config servo_config = pwm_get_default_config();
    pwm_config_set_clkdiv(&servo_config, PWM_CLOCK_DIVIDER);
    pwm_config_set_wrap(&servo_config, SERVO_FRAME_US - 1);
    pwm_init(servo_slice, &servo_config, true);
    pwm_set_gpio_level(PIN_SERVO, cart::command_to_pulse_width_us(
                                      cart::servo_command_from_correction(0.0F),
                                      SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US));

    cart::PidController pid(INITIAL_GAINS, INTEGRAL_LIMIT);
    std::uint32_t last_tick = time_us_32();
    // Reasonable startup guess until the first ultrasonic sample comes in.
    float last_good_distance_cm = cart::RAIL_LENGTH_CM / 2.0F;

    std::printf("Cart-and-rail PID controller starting\n");

    while (true) {
        const std::uint32_t now = time_us_32();
        const float dt_s = static_cast<float>(now - last_tick) / 1000000.0F;
        last_tick = now;

        // 1) GET DESIRED POSITION from the setpoint potentiometer.
        const std::uint16_t raw_pot = adc_read();
        const float desired_cm = cart::desired_position_from_pot(raw_pot, ADC_MAX);

        // 2) Actual cart position from the ultrasonic sensor. Hold the last
        //    good reading if this sample times out (e.g. sensor glitch) rather
        //    than feeding a bogus distance into the PID loop.
        float actual_cm = last_good_distance_cm;
        if (const std::optional<float> sample = measure_distance_cm(); sample.has_value()) {
            last_good_distance_cm = *sample;
            actual_cm = *sample;
        }

        // 3) PID CONTROLLER COMMAND.
        const float error = desired_cm - actual_cm;  // nominal range: -30 to +30 cm
        const cart::PidOutput out = pid.update(error, dt_s);
        const float command = cart::servo_command_from_correction(out.total);
        const std::uint16_t pulse_us =
            cart::command_to_pulse_width_us(command, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);

        pwm_set_gpio_level(PIN_SERVO, pulse_us);

        std::printf(
            "desired=%.2fcm actual=%.2fcm error=%.2fcm P=%.4f I=%.4f D=%.4f cmd=%.2f pulse=%uus\n",
            static_cast<double>(desired_cm), static_cast<double>(actual_cm),
            static_cast<double>(error), static_cast<double>(out.p), static_cast<double>(out.i),
            static_cast<double>(out.d), static_cast<double>(command),
            static_cast<unsigned>(pulse_us));

        sleep_ms(CONTROL_PERIOD_MS);
    }
}
