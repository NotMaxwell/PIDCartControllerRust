# PID Cart Controller (Rust / Embassy)

Async Rust firmware for the **Engr401 - ELT3 - Lesson 8 - Cart and Rail Lab**,
targeting a Raspberry Pi Pico (RP2040) with the
[Embassy](https://embassy.dev) async embedded framework.

The lab's original code is MicroPython; this reimplements the same control
loop in Rust/Embassy: read a setpoint potentiometer, measure the cart's
actual position, run a PID controller, and drive a servo/valve command from
the result.

## What the handout specifies

From the lab instructions PDF:

- **Setpoint input**: a potentiometer read as a raw ADC value, scaled to a
  desired cart position in cm along a 33.2cm rail, floored at 2cm:
  ```
  desired_cart_position = int(33.2 * (pot_value / 65535))
  if desired_cart_position < 2:
      desired_cart_position = 2
  ```
- **PID controller command**:
  ```
  servo_correction = P_output + I_output + D_output
  servo_command = round(servo_base_position - servo_correction, 2)
  ```
- **Expected operating ranges** (for choosing starting gains): the error
  signal `e(t)` runs roughly **-30cm to +30cm**, and the controller command
  should land around **0.54 +/- 0.36** (i.e. **0.18 to 0.9**).
- **Wiring** to the demonstrator breadboard:

  | Pico pin | Demonstrator | Purpose |
  |----------|--------------|---------|
  | GND      | GND          | common ground |
  | GPIO16   | Row 18       | controller command output |
  | GPIO2    | Row 27       | position-sensing input |
  | GPIO3    | Row 29       | position-sensing input |

## Hardware assumptions

The handout only shows two edited snippets of a larger, pre-existing
program and documents four wires. It doesn't say what's on the other end of
GPIO2/GPIO3, what the setpoint potentiometer is wired to, or exactly what
kind of servo/valve GPIO16 drives -- those were presumably established in an
earlier lesson in the series. This firmware makes the following concrete
choices, each easy to swap out if your rig differs (see "Adapting to your
hardware" below):

1. **Actual cart position**: an **HC-SR04-style ultrasonic sensor**, `TRIG`
   on GPIO2 and `ECHO` on GPIO3, mounted at one end of the rail and measuring
   distance to the cart directly in cm.
2. **Setpoint potentiometer**: wired to **GPIO26 / ADC0**, the first ADC input.
3. **Controller command output**: GPIO16 drives a **standard 1ms-2ms hobby
   servo** (positioning a valve/flap) at 50Hz. The `servo_command` in
   `0.0..=1.0` is linearly mapped onto the servo's pulse width.

## Project layout

- `src/rail.rs` / `src/pid.rs` (re-exported from `src/lib.rs`) -- pure,
  `no_std`-friendly math with no hardware dependencies: the pot-to-cm and
  correction-to-servo-command conversions ("Modify your code in two places"
  in the handout), and the PID controller itself. Unit tested on the host.
- `src/main.rs` -- the Embassy firmware: ADC read, ultrasonic ping/echo
  timing, PID loop, PWM servo output.
- `cpp/` -- a C++ port of the same firmware for comparison (see below).

## C++ port

[`cpp/`](cpp/) holds a line-for-line C++ port of this firmware built against
the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk): same
board, same wiring, same control math, written in the SDK's own idiom. It
carries the same unit tests over the same conversions and PID update.

[`cpp/README.md`](cpp/README.md) has build/test instructions and a rundown of
where the two implementations diverge -- `await` versus busy-wait, 64-bit
`Instant` versus wrapping `time_us_32()`, moved peripheral tokens versus bare
pin numbers, `defmt`-over-RTT versus `printf`-over-USB, and cargo versus
CMake-plus-an-external-SDK.

## Building and flashing

Requires the `thumbv6m-none-eabi` target (pulled in automatically via
`rust-toolchain.toml`) and either [`probe-rs`](https://probe.rs) (for a debug
probe) or [`elf2uf2-rs`](https://crates.io/crates/elf2uf2-rs) (to flash over
the Pico's USB BOOTSEL mass-storage drive).

```sh
# Debug probe (default `.cargo/config.toml` runner):
cargo run --release

# Or, over BOOTSEL USB drive instead:
cargo build --release
elf2uf2-rs -d target/thumbv6m-none-eabi/release/pid-cart-controller
```

Hold BOOTSEL while plugging in the Pico, wire it up per the table above, and
power the demonstrator breadboard before running.

`defmt` logs (desired/actual position, each PID term, and the resulting
command) print over RTT -- view them with `probe-rs` or any RTT-capable
tool.

## Testing the control math

The conversion helpers and PID controller live in a `no_std` library crate
(`#![cfg_attr(not(test), no_std)]`) so they can be unit tested on your host
machine, without touching any hardware:

```sh
cargo test --lib --target <your-host-triple>
# e.g. cargo test --lib --target x86_64-unknown-linux-gnu
```

The explicit `--target` is needed because `.cargo/config.toml` pins the
default build target to the embedded `thumbv6m-none-eabi` triple.

## Tuning the PID gains (Step 5)

`INITIAL_GAINS` in `src/main.rs` starts from a proportional-only gain sized
to the handout's expected ranges: to just reach the full +/-0.36 command
swing at the full expected +/-30cm error, `Kp ~= 0.36 / 30 = 0.012`. `Ki` and
`Kd` start at `0.0`.

Suggested optimization process, per the handout:

1. Start with `Kp = 0.012`, `Ki = Kd = 0`. Watch the logged `error` and
   `cmd` values as you move the setpoint pot.
2. Increase `Kp` until the cart starts to oscillate around the setpoint,
   then back off to roughly 60-80% of that value.
3. Add a small `Kd` to damp the oscillation/overshoot.
4. Add a small `Ki` only if a steady-state offset remains once `Kp`/`Kd` are
   dialed in -- watch for windup (the integral clamp `INTEGRAL_LIMIT` bounds
   this, but a too-large `Ki` will still cause overshoot).

## Adapting to your hardware

If your demonstrator uses different sensors:

- **Different actual-position sensor** (e.g. a linear potentiometer or a
  magnetic/quadrature encoder instead of ultrasonic): replace
  `measure_distance_cm` in `src/main.rs` with your sensor's read routine; it
  just needs to produce a `f32` in cm to feed into `error = desired_cm -
  actual_cm`.
- **Different setpoint pot pin**: change `p.PIN_26` (and the ADC channel
  construction) to whichever GPIO/ADC input it's actually wired to.
- **Different actuator** (e.g. direct PWM duty cycle to a motor driver
  instead of a hobby servo): swap `command_to_pulse_width_us` for whatever
  mapping your actuator needs, and adjust `SERVO_PULSE_MIN_US` /
  `SERVO_PULSE_MAX_US` / `SERVO_FRAME_US` accordingly.
