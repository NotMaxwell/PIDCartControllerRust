# C++ port (Raspberry Pi Pico SDK)

A line-for-line C++ port of the Embassy/Rust firmware in [`../src/`](../src/),
built against the [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk).
Same board, same wiring, same lab, same control math -- written the way you'd
write it in the SDK's own idiom, so the two implementations can be compared
side by side.

The control math is intended to be behaviorally identical: both ports carry the
same set of unit tests over the same conversions and PID update, and the tests
assert the same values.

## Layout

| C++ | Rust counterpart | Contents |
|-----|------------------|----------|
| [`include/cart/rail.hpp`](include/cart/rail.hpp) | [`src/rail.rs`](../src/rail.rs) | pot -> cm, echo -> cm, correction -> servo command, command -> pulse width |
| [`include/cart/pid.hpp`](include/cart/pid.hpp) | [`src/pid.rs`](../src/pid.rs) | the PID controller |
| [`src/main.cpp`](src/main.cpp) | [`src/main.rs`](../src/main.rs) | ADC read, ultrasonic ping/echo timing, PID loop, PWM servo output |
| [`tests/`](tests/) | `#[cfg(test)] mod tests` in each module | host unit tests for the pure math |

The pure math is header-only rather than a separate library target: it keeps
the hardware-free code hardware-free (the headers include nothing from the SDK)
while letting the host tests build with nothing but a compiler, which is the
closest analog to the Rust side's `#![cfg_attr(not(test), no_std)]` lib crate.

## Building the firmware

Needs CMake, the `arm-none-eabi` GCC toolchain, and a pico-sdk checkout:

```sh
cd cpp
cmake -B build -S . -DPICO_SDK_PATH=/path/to/pico-sdk   # or set $PICO_SDK_PATH
cmake --build build
```

That produces `build/pid_cart_controller_cpp.uf2` (drag onto the Pico's BOOTSEL
drive) alongside `build/pid_cart_controller_cpp.elf` (for `picotool`, OpenOCD,
or `probe-rs run --chip RP2040`).

Wiring is identical to the Rust firmware -- see the table in the
[top-level README](../README.md). Logs go out over **USB CDC serial** at
`printf` level; open the Pico's virtual COM port with any terminal.

## Running the tests

The tests are a standalone, SDK-free CMake project, so they build with a plain
host compiler:

```sh
cd cpp/tests
cmake -B build -S .
cmake --build build
ctest --test-dir build --output-on-failure
```

They cover the same 15 cases as the Rust `mod tests` blocks, in the same order,
with the same expected values. A few of the conversions are additionally
checked at compile time with `static_assert`.

## Where the two ports differ

The control math is the same; almost everything around it is not. That's the
interesting part of the comparison.

### Concurrency: `await` vs. busy-wait

This is the biggest behavioral difference, and it's the whole reason the Rust
version pulls in Embassy.

- **Rust** `awaits` everything that waits. `echo.wait_for_high()` registers a
  GPIO interrupt and yields to the executor; `Timer::after(CONTROL_PERIOD)`
  arms a hardware timer and yields. Between samples the CPU is genuinely idle
  and available -- adding a second concurrent task (a display, a UART command
  interface, telemetry) means spawning it, with no restructuring of the control
  loop.
- **C++** polls. `wait_for_level()` spins on `gpio_get()` until the pin flips or
  the timeout expires, and `sleep_ms()` blocks the only thread of control.
  Simple and perfectly adequate for one loop, but the ~30ms worst-case echo
  wait is 30ms of the CPU doing nothing else, and a second concurrent activity
  means interrupts, a second core, or restructuring into a state machine.

For *this* lab, with one loop and nothing else to do, the busy-wait costs
nothing real. The difference shows up the moment the firmware grows.

### Timekeeping and wraparound

Embassy's `Instant` is a 64-bit tick count that will not wrap in any practical
runtime. The SDK's `time_us_32()` wraps every ~71.6 minutes, so the C++ port
computes every elapsed time as an unsigned subtraction (`now - start`), which
stays correct across the wrap. Using `time_us_64()` would sidestep it entirely
at the cost of 64-bit math in the hot path -- the unsigned-subtraction idiom is
the conventional SDK answer, but it *is* something the C++ author has to know
and get right, and the Rust author never thinks about.

### Peripheral ownership

`embassy_rp::init()` hands back a struct of singleton peripheral tokens that
are *moved* into the drivers that use them. `Pwm::new_output_a(p.PWM_SLICE0,
p.PIN_16, ...)` consumes `PIN_16`; a second attempt to use that pin elsewhere
is a compile error.

In the C++ port pins are `constexpr uint32_t` constants and the SDK's functions
take pin numbers. Nothing prevents configuring GPIO16 as PWM in one place and
as a plain output in another -- it's a runtime bug on real hardware. The
constants at the top of `main.cpp` are the mitigation, not a guarantee.

### Failure signalling

`measure_distance_cm` returns `Option<f32>` / `std::optional<float>` -- a rare
spot where the two languages line up exactly, and both callers must unwrap
before using a distance. Elsewhere they diverge: the Rust ADC read is a
`Result` that the firmware explicitly handles with `unwrap_or(0)`, while
`adc_read()` just returns a value with no error channel; and a Rust panic lands
in `panic-probe` and prints over RTT, where C++ has no equivalent backstop.

### Logging

`defmt` sends *format-string IDs plus raw arguments* over RTT and does the
formatting on the host, so a log line costs a handful of bytes on the wire, no
float-formatting code on the device, and no USB stack. The C++ port uses
`printf` over USB CDC, which is far easier to read with any terminal program
and needs no special tooling -- at the cost of linking the full floating-point
`printf` and the TinyUSB device stack into the image, and doing the formatting
inline in the control loop.

### Build and test workflow

- **Rust**: `cargo build --release`. The target triple, the linker script
  wiring, and the RP2040 second-stage bootloader all come from
  `rust-toolchain.toml`, `build.rs`, and `embassy-rp` respectively. Host tests
  are `cargo test --lib --target <host-triple>` with no extra scaffolding, and
  dependencies resolve themselves.
- **C++**: CMake plus an out-of-band pico-sdk checkout located via
  `PICO_SDK_PATH`, plus an `arm-none-eabi` toolchain you install separately.
  Host tests need a *second* CMake project (the first one cross-compiles
  everything) and a test harness -- here a ~50-line hand-rolled one in
  `tests/check.hpp`, to match the Rust side's zero dependencies; a real project
  would likely reach for Catch2 or GoogleTest.

### Compile-time evaluation

Both ports evaluate the conversion math at compile time -- Rust with `const fn`
and C++ with `constexpr` -- and the C++ tests use `static_assert` to pin a few
headline conversions at compile time. The practical difference is in the
numeric conversions themselves: Rust's `as` casts are explicit and
non-optional, while C++ will silently convert between `float`, `int`, and
`uint16_t`, which is why the C++ headers spell out every `static_cast`.

## A note on the integration scheme

Both ports accumulate the integral term as `integral += error * dt` -- that is
rectangular (forward-Euler) integration, which is what the Rust code does even
though its doc comment says "trapezoidal". The C++ header describes it as
forward-Euler to match the actual behavior. Changing the scheme is a real
control-design decision, so this port mirrors the existing behavior rather than
"fixing" it; if you do switch to trapezoidal
(`integral += 0.5 * (error + prev_error) * dt`), change both ports and the
Rust doc comment together.
