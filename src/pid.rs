//! Textbook PID controller matching the block diagram in the lab handout:
//!
//! ```text
//! e(t) --+--> [ Kp * e(t)            ] --+
//!        +--> [ Ki * integral(e(t)) ] --+--> controller command
//!        +--> [ Kd * d(e(t))/dt      ] --+
//! ```

/// Proportional / integral / derivative gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidGains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

/// The individual P/I/D contributions plus their sum, so a caller can log
/// each term while tuning gains (Step 5 of the lab handout).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PidOutput {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub total: f32,
}

/// A discrete-time PID controller with trapezoidal integration and an
/// anti-windup clamp on the accumulated integral term.
#[derive(Clone, Copy, Debug)]
pub struct PidController {
    gains: PidGains,
    integral: f32,
    integral_limit: f32,
    prev_error: f32,
    has_prev_error: bool,
}

impl PidController {
    pub const fn new(gains: PidGains, integral_limit: f32) -> Self {
        Self {
            gains,
            integral: 0.0,
            integral_limit,
            prev_error: 0.0,
            has_prev_error: false,
        }
    }

    /// Replace the gains in place (handy while live-tuning).
    pub fn set_gains(&mut self, gains: PidGains) {
        self.gains = gains;
    }

    pub fn gains(&self) -> PidGains {
        self.gains
    }

    /// Clear accumulated integral / derivative history, e.g. after a setpoint jump.
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
        self.has_prev_error = false;
    }

    /// Advance the controller by one sample.
    ///
    /// * `error` — `desired - actual`, in the same units as the plant (cm on the rail).
    /// * `dt_s` — seconds elapsed since the previous call.
    pub fn update(&mut self, error: f32, dt_s: f32) -> PidOutput {
        let p = self.gains.kp * error;

        if dt_s > 0.0 {
            self.integral += error * dt_s;
            self.integral = self
                .integral
                .clamp(-self.integral_limit, self.integral_limit);
        }
        let i = self.gains.ki * self.integral;

        let derivative = if self.has_prev_error && dt_s > 0.0 {
            (error - self.prev_error) / dt_s
        } else {
            0.0
        };
        let d = self.gains.kd * derivative;

        self.prev_error = error;
        self.has_prev_error = true;

        PidOutput {
            p,
            i,
            d,
            total: p + i + d,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gains(kp: f32, ki: f32, kd: f32) -> PidGains {
        PidGains { kp, ki, kd }
    }

    #[test]
    fn proportional_only_scales_error() {
        let mut pid = PidController::new(gains(0.5, 0.0, 0.0), 100.0);
        let out = pid.update(10.0, 0.05);
        assert_eq!(out.p, 5.0);
        assert_eq!(out.i, 0.0);
        assert_eq!(out.d, 0.0);
        assert_eq!(out.total, 5.0);
    }

    #[test]
    fn integral_accumulates_over_time() {
        let mut pid = PidController::new(gains(0.0, 1.0, 0.0), 100.0);
        pid.update(2.0, 1.0); // integral = 2.0
        let out = pid.update(2.0, 1.0); // integral = 4.0
        assert_eq!(out.i, 4.0);
    }

    #[test]
    fn integral_is_clamped_for_anti_windup() {
        let mut pid = PidController::new(gains(0.0, 1.0, 0.0), 3.0);
        pid.update(10.0, 1.0);
        let out = pid.update(10.0, 1.0);
        // Raw integral would be 20.0, but must clamp to the +-3.0 limit.
        assert_eq!(out.i, 3.0);
    }

    #[test]
    fn derivative_is_zero_on_first_sample() {
        let mut pid = PidController::new(gains(0.0, 0.0, 1.0), 100.0);
        let out = pid.update(5.0, 0.1);
        assert_eq!(out.d, 0.0);
    }

    #[test]
    fn derivative_tracks_rate_of_change() {
        let mut pid = PidController::new(gains(0.0, 0.0, 2.0), 100.0);
        pid.update(0.0, 0.1);
        let out = pid.update(1.0, 0.1); // d(error)/dt = 1.0 / 0.1 = 10.0
        assert_eq!(out.d, 20.0);
    }

    #[test]
    fn reset_clears_history() {
        let mut pid = PidController::new(gains(0.0, 1.0, 1.0), 100.0);
        pid.update(5.0, 1.0);
        pid.reset();
        let out = pid.update(5.0, 1.0);
        // Integral restarted from 0 and derivative has no previous sample.
        assert_eq!(out.i, 5.0);
        assert_eq!(out.d, 0.0);
    }

    #[test]
    fn zero_dt_does_not_change_integral_or_blow_up_derivative() {
        let mut pid = PidController::new(gains(0.0, 1.0, 1.0), 100.0);
        pid.update(5.0, 1.0);
        let out = pid.update(5.0, 0.0);
        assert_eq!(out.i, 5.0); // unchanged
        assert_eq!(out.d, 0.0); // guarded against division by zero
    }
}
