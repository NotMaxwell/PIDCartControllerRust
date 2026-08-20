//! Embassy firmware for the Engr401 ELT3 "Cart and Rail" PID lab.
//!
//! Target board: Raspberry Pi Pico (RP2040).
//!
//! Wiring, per the lab handout:
//!   GND         -> demonstrator GND
//!   GPIO16      -> demonstrator Row 18   (servo/valve PWM control signal)
//!   GPIO2       -> demonstrator Row 27   (ultrasonic sensor TRIG)
//!   GPIO3       -> demonstrator Row 29   (ultrasonic sensor ECHO)
//!   GPIO26/ADC0 -> setpoint potentiometer wiper
//!
//! The handout only documents the four demonstrator-breadboard wires
//! (GND, GPIO16, GPIO2, GPIO3) since the setpoint potentiometer and the rest
//! of the Pico wiring were presumably set up in an earlier lesson in the
//! series. This firmware makes two concrete, clearly-labelled assumptions
//! about hardware that isn't pinned down by the handout alone -- see the
//! "Hardware assumptions" section of README.md before wiring up:
//!   1. Actual cart position is sensed with an HC-SR04-style ultrasonic
//!      sensor (TRIG on GPIO2, ECHO on GPIO3).
//!   2. The controller command drives a standard 1-2ms hobby servo (valve
//!      or flap) on GPIO16, with the 0.0..=1.0 command linearly mapped onto
//!      the servo's pulse width.
//! Both are easy to swap out for different hardware -- see the constants and
//! `measure_distance_cm` below.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{
    Adc, Channel as AdcChannel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler,
};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::{with_timeout, Duration, Instant, Timer};
use fixed::types::extra::U4;
use fixed::FixedU16;
use panic_probe as _;

use pid_cart_controller::{
    command_to_pulse_width_us, desired_position_from_pot, distance_from_echo_micros,
    servo_command_from_correction, PidController, PidGains, RAIL_LENGTH_CM,
};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

// Note: embassy-rp embeds the RP2040 second-stage bootloader (`.boot2`)
// itself by default (see its `boot2-*` Cargo features), so there's no need
// to provide one here.

/// Control loop rate. 20 Hz is comfortably faster than the cart's mechanical
/// dynamics and leaves plenty of headroom for the ultrasonic sensor's
/// ~30ms round-trip time.
const CONTROL_PERIOD: Duration = Duration::from_millis(50);

/// RP2040 ADC resolution (12-bit).
const ADC_MAX: u16 = 4095;

/// Standard hobby-servo pulse-width range (1ms-2ms within a 20ms/50Hz frame).
const SERVO_PULSE_MIN_US: u16 = 1_000;
const SERVO_PULSE_MAX_US: u16 = 2_000;
const SERVO_FRAME_US: u16 = 20_000;

/// How long to wait for an ultrasonic echo before giving up on a sample.
/// ~30ms corresponds to roughly a 5m round trip, well beyond the rail length.
const ULTRASONIC_TIMEOUT: Duration = Duration::from_millis(30);

/// Starting PID gains for Step 5 of the lab handout ("determine reasonable
/// PID gains to begin your optimization process").
///
/// The handout gives the expected operating ranges: error e(t) spans roughly
/// +-30cm and the controller command needs to swing +-0.36 around its 0.54
/// base. A proportional-only gain that can just reach the full command swing
/// at the full expected error is a reasonable, physically-motivated starting
/// point:
///     Kp ~= 0.36 / 30cm = 0.012
/// Start tuning from here: increase Kp until the cart starts to oscillate
/// around the setpoint, back off a bit, then add a small Kd to damp the
/// oscillation, and finally a small Ki to remove any steady-state error.
const INITIAL_GAINS: PidGains = PidGains {
    kp: 0.012,
    ki: 0.0,
    kd: 0.0,
};

/// Anti-windup clamp on the accumulated integral term (cm*s). Prevents the
/// integral from running away while Ki is still being tuned up from zero.
const INTEGRAL_LIMIT: f32 = 15.0;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // --- Setpoint potentiometer (desired cart position) ---
    let mut adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    let mut pot_channel = AdcChannel::new_pin(p.PIN_26, Pull::None);

    // --- Ultrasonic range sensor (actual cart position) ---
    let mut trig = Output::new(p.PIN_2, Level::Low);
    let mut echo = Input::new(p.PIN_3, Pull::None);

    // --- Servo/valve PWM output ---
    // System clock is 125MHz by default; dividing by 125 gives a 1MHz PWM
    // counter, i.e. 1 count = 1us, so `top`/`compare_a` can be set directly
    // in microseconds.
    let mut pwm_config = PwmConfig::default();
    pwm_config.divider = FixedU16::<U4>::from_num(125);
    pwm_config.top = SERVO_FRAME_US - 1;
    pwm_config.compare_a = command_to_pulse_width_us(
        servo_command_from_correction(0.0),
        SERVO_PULSE_MIN_US,
        SERVO_PULSE_MAX_US,
    );
    // GPIO16 maps to PWM slice 0, channel A on the RP2040.
    let mut servo_pwm = Pwm::new_output_a(p.PWM_SLICE0, p.PIN_16, pwm_config.clone());

    let mut pid = PidController::new(INITIAL_GAINS, INTEGRAL_LIMIT);
    let mut last_tick = Instant::now();
    // Reasonable startup guess until the first ultrasonic sample comes in.
    let mut last_good_distance_cm = RAIL_LENGTH_CM / 2.0;

    info!("Cart-and-rail PID controller starting");

    loop {
        let now = Instant::now();
        let dt_s = (now - last_tick).as_micros() as f32 / 1_000_000.0;
        last_tick = now;

        // 1) GET DESIRED POSITION from the setpoint potentiometer.
        let raw_pot = adc.read(&mut pot_channel).await.unwrap_or(0);
        let desired_cm = desired_position_from_pot(raw_pot, ADC_MAX);

        // 2) Actual cart position from the ultrasonic sensor. Hold the last
        //    good reading if this sample times out (e.g. sensor glitch)
        //    rather than feeding a bogus distance into the PID loop.
        let actual_cm = match measure_distance_cm(&mut trig, &mut echo).await {
            Some(cm) => {
                last_good_distance_cm = cm;
                cm
            }
            None => last_good_distance_cm,
        };

        // 3) PID CONTROLLER COMMAND.
        let error = desired_cm - actual_cm; // nominal range: -30 to +30 cm
        let out = pid.update(error, dt_s);
        let command = servo_command_from_correction(out.total);
        let pulse_us = command_to_pulse_width_us(command, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);

        pwm_config.compare_a = pulse_us;
        servo_pwm.set_config(&pwm_config);

        info!(
            "desired={=f32}cm actual={=f32}cm error={=f32}cm P={=f32} I={=f32} D={=f32} cmd={=f32} pulse={=u16}us",
            desired_cm, actual_cm, error, out.p, out.i, out.d, command, pulse_us
        );

        Timer::after(CONTROL_PERIOD).await;
    }
}

/// Trigger an HC-SR04-style ultrasonic sensor and time its echo pulse to get
/// a distance in centimeters. Returns `None` on timeout (no echo received --
/// e.g. the cart is out of range or the sensor isn't connected yet).
async fn measure_distance_cm(trig: &mut Output<'_>, echo: &mut Input<'_>) -> Option<f32> {
    trig.set_high();
    Timer::after(Duration::from_micros(10)).await;
    trig.set_low();

    with_timeout(ULTRASONIC_TIMEOUT, echo.wait_for_high())
        .await
        .ok()?;
    let start = Instant::now();

    with_timeout(ULTRASONIC_TIMEOUT, echo.wait_for_low())
        .await
        .ok()?;
    let echo_us = (Instant::now() - start).as_micros() as u32;

    Some(distance_from_echo_micros(echo_us))
}
