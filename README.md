# Vehicle Controller

## Getting Started

Install dependencies

```shell
# Install tools
cargo install probe-rs flip-link
# Add target platform
rustup target add thumbv7em-none-eabihf
```

Once you have connected your programmer to the target, you can run `cargo run` like any other rust project.

### Unit Tests

Surprisingly, just because we're on an embedded platform, we still have the ability to run unit tests on the target, thanks to [`defmt-test`](https://crates.io/crates/defmt-test).

Just use `cargo test` to run each unit test module (each is flashed and run separately).

### Log Level

The logging level use by defmt can be set with the environment variable `DEFMT_LOG`. Possible values are `trace`, `debug`, `info`, `warn`, `error`, and `off`. By default the log level is set to `info`.

## References

- [RM0399 STM32H745/755 and STM32H747/757 reference manual ](https://www.st.com/resource/en/reference_manual/rm0399-stm32h745755-and-stm32h747757-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [RTIC](https://rtic.rs/2/book/en/)
- [defmt](https://defmt.ferrous-systems.com/)
