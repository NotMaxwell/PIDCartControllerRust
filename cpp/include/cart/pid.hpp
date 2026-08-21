// Textbook PID controller matching the block diagram in the lab handout:
//
//   e(t) --+--> [ Kp * e(t)           ] --+
//          +--> [ Ki * integral(e(t)) ] --+--> controller command
//          +--> [ Kd * d(e(t))/dt     ] --+
//
// C++ port of `src/pid.rs`. Header-only and free of any SDK/hardware types so
// the same code compiles for the RP2040 firmware and for the host unit tests.

#ifndef CART_PID_HPP
#define CART_PID_HPP

#include <algorithm>

namespace cart {

// Proportional / integral / derivative gains.
struct PidGains {
    float kp;
    float ki;
    float kd;
};

// The individual P/I/D contributions plus their sum, so a caller can log each
// term while tuning gains (Step 5 of the lab handout).
struct PidOutput {
    float p = 0.0F;
    float i = 0.0F;
    float d = 0.0F;
    float total = 0.0F;
};

// A discrete-time PID controller with forward-Euler integration and an
// anti-windup clamp on the accumulated integral term.
class PidController {
public:
    constexpr PidController(PidGains gains, float integral_limit)
        : gains_(gains), integral_limit_(integral_limit) {}

    // Replace the gains in place (handy while live-tuning).
    constexpr void set_gains(PidGains gains) { gains_ = gains; }

    [[nodiscard]] constexpr PidGains gains() const { return gains_; }

    // Clear accumulated integral / derivative history, e.g. after a setpoint jump.
    constexpr void reset() {
        integral_ = 0.0F;
        prev_error_ = 0.0F;
        has_prev_error_ = false;
    }

    // Advance the controller by one sample.
    //
    //   error - `desired - actual`, in the same units as the plant (cm on the rail).
    //   dt_s  - seconds elapsed since the previous call.
    constexpr PidOutput update(float error, float dt_s) {
        const float p = gains_.kp * error;

        if (dt_s > 0.0F) {
            integral_ += error * dt_s;
            integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
        }
        const float i = gains_.ki * integral_;

        const float derivative =
            (has_prev_error_ && dt_s > 0.0F) ? (error - prev_error_) / dt_s : 0.0F;
        const float d = gains_.kd * derivative;

        prev_error_ = error;
        has_prev_error_ = true;

        return PidOutput{p, i, d, p + i + d};
    }

private:
    PidGains gains_;
    float integral_ = 0.0F;
    float integral_limit_;
    float prev_error_ = 0.0F;
    bool has_prev_error_ = false;
};

}  // namespace cart

#endif  // CART_PID_HPP
