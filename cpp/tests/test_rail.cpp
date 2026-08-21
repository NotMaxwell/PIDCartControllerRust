// Host unit tests for cart/rail.hpp -- a one-to-one port of the `mod tests` in
// `src/rail.rs`, so a change to the conversion math shows up on both sides.

#include "cart/rail.hpp"

#include "check.hpp"

using cart::command_to_pulse_width_us;
using cart::desired_position_from_pot;
using cart::distance_from_echo_micros;
using cart::servo_command_from_correction;

TEST(pot_at_zero_floors_to_min_position) {
    CHECK_EQ(desired_position_from_pot(0, 4095), cart::MIN_DESIRED_POSITION_CM);
}

TEST(pot_at_full_scale_reaches_rail_length) {
    CHECK_NEAR(desired_position_from_pot(4095, 4095), cart::RAIL_LENGTH_CM, 0.01);
}

TEST(pot_at_half_scale_is_half_rail_length) {
    CHECK_NEAR(desired_position_from_pot(2048, 4095), cart::RAIL_LENGTH_CM / 2.0F, 0.05);
}

TEST(echo_pulse_converts_to_expected_distance) {
    // 58us round trip ~= 1cm.
    CHECK_NEAR(distance_from_echo_micros(58), 1.0, 0.01);
    CHECK_NEAR(distance_from_echo_micros(580), 10.0, 0.01);
}

TEST(servo_command_centers_on_base_position_with_zero_correction) {
    CHECK_EQ(servo_command_from_correction(0.0F), cart::SERVO_BASE_POSITION);
}

TEST(servo_command_clamps_to_safe_range) {
    CHECK_EQ(servo_command_from_correction(10.0F), cart::SERVO_COMMAND_MIN);
    CHECK_EQ(servo_command_from_correction(-10.0F), cart::SERVO_COMMAND_MAX);
}

TEST(pulse_width_maps_command_range_to_servo_range) {
    CHECK_EQ(command_to_pulse_width_us(0.0F, 1000, 2000), 1000);
    CHECK_EQ(command_to_pulse_width_us(1.0F, 1000, 2000), 2000);
    CHECK_EQ(command_to_pulse_width_us(0.5F, 1000, 2000), 1500);
}

TEST(pulse_width_clamps_out_of_range_commands) {
    CHECK_EQ(command_to_pulse_width_us(-1.0F, 1000, 2000), 1000);
    CHECK_EQ(command_to_pulse_width_us(2.0F, 1000, 2000), 2000);
}

// The conversions are `constexpr`, so the compiler can check the headline cases
// at compile time -- the closest C++ equivalent of the guarantees the Rust port
// gets from `const fn` plus its stricter numeric conversions.
static_assert(desired_position_from_pot(0, 4095) == cart::MIN_DESIRED_POSITION_CM);
static_assert(servo_command_from_correction(0.0F) == cart::SERVO_BASE_POSITION);
static_assert(command_to_pulse_width_us(0.5F, 1000, 2000) == 1500);
