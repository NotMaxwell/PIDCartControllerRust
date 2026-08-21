// Host unit tests for cart/pid.hpp -- a one-to-one port of the `mod tests` in
// `src/pid.rs`, so a change to the control math shows up on both sides.

#include "cart/pid.hpp"

#include "check.hpp"

using cart::PidController;
using cart::PidGains;
using cart::PidOutput;

namespace {
constexpr PidGains gains(float kp, float ki, float kd) { return PidGains{kp, ki, kd}; }
}  // namespace

TEST(proportional_only_scales_error) {
    PidController pid(gains(0.5F, 0.0F, 0.0F), 100.0F);
    const PidOutput out = pid.update(10.0F, 0.05F);
    CHECK_EQ(out.p, 5.0);
    CHECK_EQ(out.i, 0.0);
    CHECK_EQ(out.d, 0.0);
    CHECK_EQ(out.total, 5.0);
}

TEST(integral_accumulates_over_time) {
    PidController pid(gains(0.0F, 1.0F, 0.0F), 100.0F);
    pid.update(2.0F, 1.0F);                       // integral = 2.0
    const PidOutput out = pid.update(2.0F, 1.0F); // integral = 4.0
    CHECK_EQ(out.i, 4.0);
}

TEST(integral_is_clamped_for_anti_windup) {
    PidController pid(gains(0.0F, 1.0F, 0.0F), 3.0F);
    pid.update(10.0F, 1.0F);
    const PidOutput out = pid.update(10.0F, 1.0F);
    // Raw integral would be 20.0, but must clamp to the +-3.0 limit.
    CHECK_EQ(out.i, 3.0);
}

TEST(derivative_is_zero_on_first_sample) {
    PidController pid(gains(0.0F, 0.0F, 1.0F), 100.0F);
    const PidOutput out = pid.update(5.0F, 0.1F);
    CHECK_EQ(out.d, 0.0);
}

TEST(derivative_tracks_rate_of_change) {
    PidController pid(gains(0.0F, 0.0F, 2.0F), 100.0F);
    pid.update(0.0F, 0.1F);
    const PidOutput out = pid.update(1.0F, 0.1F); // d(error)/dt = 1.0 / 0.1 = 10.0
    CHECK_EQ(out.d, 20.0);
}

TEST(reset_clears_history) {
    PidController pid(gains(0.0F, 1.0F, 1.0F), 100.0F);
    pid.update(5.0F, 1.0F);
    pid.reset();
    const PidOutput out = pid.update(5.0F, 1.0F);
    // Integral restarted from 0 and derivative has no previous sample.
    CHECK_EQ(out.i, 5.0);
    CHECK_EQ(out.d, 0.0);
}

TEST(zero_dt_does_not_change_integral_or_blow_up_derivative) {
    PidController pid(gains(0.0F, 1.0F, 1.0F), 100.0F);
    pid.update(5.0F, 1.0F);
    const PidOutput out = pid.update(5.0F, 0.0F);
    CHECK_EQ(out.i, 5.0); // unchanged
    CHECK_EQ(out.d, 0.0); // guarded against division by zero
}
