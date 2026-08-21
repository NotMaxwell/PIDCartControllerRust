// Pure conversion helpers for the cart-and-rail I/O, mirroring the two spots
// the lab handout has students edit in the original code:
//
//   # GET DESIRED POSITION
//   pot_value = pot_pin.read_u16()
//   desired_cart_position = int(33.2*(pot_value/65535))   # [cm]
//   if desired_cart_position < 2:
//       desired_cart_position = 2
//
//   # PID CONTROLLER COMMAND
//   servo_correction = P_output + I_output + D_output
//   servo_command = round(servo_base_position - servo_correction, 2)
//
// C++ port of `src/rail.rs`. Header-only and free of any SDK/hardware types so
// it can be exercised by the host unit tests in `cpp/tests`, independent of the
// pico-sdk firmware in `cpp/src/main.cpp`.

#ifndef CART_RAIL_HPP
#define CART_RAIL_HPP

#include <algorithm>
#include <cstdint>

namespace cart {

// Physical length of the rail in centimeters (the `33.2` in the original code).
inline constexpr float RAIL_LENGTH_CM = 33.2F;

// Floor on the commanded setpoint, matching
// `if desired_cart_position < 2: desired_cart_position = 2`.
inline constexpr float MIN_DESIRED_POSITION_CM = 2.0F;

// Neutral/base servo command the PID correction is applied around
// (`servo_base_position` in the original code). Per the handout, the
// controller command is expected to sit at 0.54 +/- 0.36.
inline constexpr float SERVO_BASE_POSITION = 0.54F;

// Hardware-safety clamp for the final servo command, comfortably wider than
// the handout's expected operating range of 0.18 to 0.9.
inline constexpr float SERVO_COMMAND_MIN = 0.05F;
inline constexpr float SERVO_COMMAND_MAX = 0.95F;

// Convert a raw ADC sample from the setpoint potentiometer into a desired cart
// position in cm.
//
// `raw` is the ADC reading and `adc_max` is the value a full-scale reading
// would produce (e.g. 4095 for the RP2040's 12-bit ADC, or 65535 if you scale
// up to match the original 16-bit `read_u16()` reading).
[[nodiscard]] constexpr float desired_position_from_pot(std::uint16_t raw, std::uint16_t adc_max) {
    const float ratio = static_cast<float>(raw) / static_cast<float>(adc_max);
    const float cm = RAIL_LENGTH_CM * ratio;
    return cm < MIN_DESIRED_POSITION_CM ? MIN_DESIRED_POSITION_CM : cm;
}

// Convert an HC-SR04-style ultrasonic sensor's echo pulse width (microseconds)
// into a distance in centimeters, using the standard round-trip approximation
// of ~58us per cm (speed of sound ~343 m/s).
[[nodiscard]] constexpr float distance_from_echo_micros(std::uint32_t echo_us) {
    return static_cast<float>(echo_us) / 58.0F;
}

// Combine the neutral servo position and the PID correction into a servo
// command, then clamp it to a hardware-safe range.
[[nodiscard]] constexpr float servo_command_from_correction(float correction) {
    const float raw = SERVO_BASE_POSITION - correction;
    return std::clamp(raw, SERVO_COMMAND_MIN, SERVO_COMMAND_MAX);
}

// Map a normalized servo command (0.0..=1.0, nominally 0.18..=0.9 per the
// handout) onto a PWM pulse width in microseconds for a standard hobby servo
// (1ms..2ms pulse within a 20ms/50Hz frame).
[[nodiscard]] constexpr std::uint16_t command_to_pulse_width_us(float command, std::uint16_t min_us,
                                                                std::uint16_t max_us) {
    const float clamped = std::clamp(command, 0.0F, 1.0F);
    const float span = static_cast<float>(max_us - min_us);
    // `clamped * span` is always >= 0.0 here, so adding 0.5 before truncating
    // rounds to the nearest microsecond (and keeps this usable in constexpr,
    // where <cmath>'s std::round is not guaranteed to be).
    return static_cast<std::uint16_t>(min_us + static_cast<std::uint16_t>(clamped * span + 0.5F));
}

}  // namespace cart

#endif  // CART_RAIL_HPP
