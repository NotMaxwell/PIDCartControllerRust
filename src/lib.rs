#![cfg_attr(not(test), no_std)]
//! Core control-loop math for the Engr401 ELT3 "Cart and Rail" PID lab.
//!
//! This is split out from `src/main.rs` so the pure math (position scaling,
//! the PID controller, and the servo command mapping) can be unit tested on
//! the host with plain `cargo test`, independent of the Embassy/RP2040
//! firmware in `main.rs` that drives the real hardware. See `pid.rs` and
//! `rail.rs` for the two spots the lab handout has students fill in.

mod pid;
mod rail;

pub use pid::{PidController, PidGains, PidOutput};
pub use rail::{
    command_to_pulse_width_us, desired_position_from_pot, distance_from_echo_micros,
    servo_command_from_correction, MIN_DESIRED_POSITION_CM, RAIL_LENGTH_CM, SERVO_BASE_POSITION,
    SERVO_COMMAND_MAX, SERVO_COMMAND_MIN,
};
