//! Pure conversion helpers for the cart-and-rail I/O, mirroring the two spots
//! the lab handout has students edit in the original code:
//!
//! ```text
//! # GET DESIRED POSITION
//! pot_value = pot_pin.read_u16()
//! desired_cart_position = int(33.2*(pot_value/65535))   # [cm]
//! if desired_cart_position < 2:
//!     desired_cart_position = 2
//!
//! # PID CONTROLLER COMMAND
//! servo_correction = P_output + I_output + D_output
//! servo_command = round(servo_base_position - servo_correction, 2)
//! ```
//!
//! Kept free of any hardware/HAL types so it can be exercised with plain
//! `cargo test` on the host, independent of the Embassy/RP2040 firmware.

/// Physical length of the rail in centimeters (the `33.2` in the original code).
pub const RAIL_LENGTH_CM: f32 = 33.2;

/// Floor on the commanded setpoint, matching
/// `if desired_cart_position < 2: desired_cart_position = 2`.
pub const MIN_DESIRED_POSITION_CM: f32 = 2.0;

/// Neutral/base servo command the PID correction is applied around
/// (`servo_base_position` in the original code). Per the handout, the
/// controller command is expected to sit at 0.54 +/- 0.36.
pub const SERVO_BASE_POSITION: f32 = 0.54;

/// Hardware-safety clamp for the final servo command, comfortably wider than
/// the handout's expected operating range of 0.18 to 0.9.
pub const SERVO_COMMAND_MIN: f32 = 0.05;
pub const SERVO_COMMAND_MAX: f32 = 0.95;

/// Convert a raw ADC sample from the setpoint potentiometer into a desired
/// cart position in cm.
///
/// `raw` is the ADC reading and `adc_max` is the value a full-scale reading
/// would produce (e.g. `4095` for the RP2040's 12-bit ADC, or `65535` if you
/// scale up to match the original 16-bit `read_u16()` reading).
pub fn desired_position_from_pot(raw: u16, adc_max: u16) -> f32 {
    let ratio = raw as f32 / adc_max as f32;
    let cm = RAIL_LENGTH_CM * ratio;
    if cm < MIN_DESIRED_POSITION_CM {
        MIN_DESIRED_POSITION_CM
    } else {
        cm
    }
}

/// Convert an HC-SR04-style ultrasonic sensor's echo pulse width (microseconds)
/// into a distance in centimeters, using the standard round-trip approximation
/// of ~58us per cm (speed of sound ~343 m/s).
pub fn distance_from_echo_micros(echo_us: u32) -> f32 {
    echo_us as f32 / 58.0
}

/// Combine the neutral servo position and the PID correction into a servo
/// command, then clamp it to a hardware-safe range.
pub fn servo_command_from_correction(correction: f32) -> f32 {
    let raw = SERVO_BASE_POSITION - correction;
    raw.clamp(SERVO_COMMAND_MIN, SERVO_COMMAND_MAX)
}

/// Map a normalized servo command (0.0..=1.0, nominally 0.18..=0.9 per the
/// handout) onto a PWM pulse width in microseconds for a standard hobby servo
/// (1ms..2ms pulse within a 20ms/50Hz frame).
pub fn command_to_pulse_width_us(command: f32, min_us: u16, max_us: u16) -> u16 {
    let clamped = command.clamp(0.0, 1.0);
    let span = (max_us - min_us) as f32;
    // `f32::round` isn't available in `core` without `libm`; `clamped * span`
    // is always >= 0.0 here, so adding 0.5 before truncating rounds to the
    // nearest microsecond just as well.
    min_us + (clamped * span + 0.5) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pot_at_zero_floors_to_min_position() {
        assert_eq!(desired_position_from_pot(0, 4095), MIN_DESIRED_POSITION_CM);
    }

    #[test]
    fn pot_at_full_scale_reaches_rail_length() {
        let cm = desired_position_from_pot(4095, 4095);
        assert!((cm - RAIL_LENGTH_CM).abs() < 0.01);
    }

    #[test]
    fn pot_at_half_scale_is_half_rail_length() {
        let cm = desired_position_from_pot(2048, 4095);
        assert!((cm - RAIL_LENGTH_CM / 2.0).abs() < 0.05);
    }

    #[test]
    fn echo_pulse_converts_to_expected_distance() {
        // 58us round trip ~= 1cm.
        assert!((distance_from_echo_micros(58) - 1.0).abs() < 0.01);
        assert!((distance_from_echo_micros(580) - 10.0).abs() < 0.01);
    }

    #[test]
    fn servo_command_centers_on_base_position_with_zero_correction() {
        assert_eq!(servo_command_from_correction(0.0), SERVO_BASE_POSITION);
    }

    #[test]
    fn servo_command_clamps_to_safe_range() {
        assert_eq!(servo_command_from_correction(10.0), SERVO_COMMAND_MIN);
        assert_eq!(servo_command_from_correction(-10.0), SERVO_COMMAND_MAX);
    }

    #[test]
    fn pulse_width_maps_command_range_to_servo_range() {
        assert_eq!(command_to_pulse_width_us(0.0, 1000, 2000), 1000);
        assert_eq!(command_to_pulse_width_us(1.0, 1000, 2000), 2000);
        assert_eq!(command_to_pulse_width_us(0.5, 1000, 2000), 1500);
    }

    #[test]
    fn pulse_width_clamps_out_of_range_commands() {
        assert_eq!(command_to_pulse_width_us(-1.0, 1000, 2000), 1000);
        assert_eq!(command_to_pulse_width_us(2.0, 1000, 2000), 2000);
    }
}
