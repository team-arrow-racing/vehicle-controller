# Vehicle Controller

Responstivle for controlling the main function of the vehicle.

## Setup

```shell
# Add cross-compilation target
rustup target add thumbv7em-none-eabi
# Install probe-run
cargo install probe-run
```

On Linux you may need to configure your udev rules to allow running without root.

## Running

The following will compile, flash and debug the program.

```shell
DEFMT_LOG=info cargo run
```

## References

- [Real-Time Interrupt-driven Concurrency Framework](https://rtic.rs/1/)
- [defmt](https://defmt.ferrous-systems.com/)
- [probe-run](https://github.com/knurling-rs/probe-run)
- [Rust HAL Documentation](https://docs.rs/stm32l4xx-hal/latest/stm32l4xx_hal/)
- [STM32L433xx Datasheet](https://www.st.com/resource/en/datasheet/stm32l433cc.pdf) 
- [RM0394 Reference Manual for STM32L41xxx/42xxx/43xxx/44xxx/45xxx/46xxx](https://www.st.com/resource/en/reference_manual/dm00151940-stm32l41xxx42xxx43xxx44xxx45xxx46xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [Calypso Wi-Fi Module Reference](https://www.we-online.com/components/products/manual/2610011025000_Calypso%20261001102500x%20Manual_rev2.0.pdf)

## About Team Arrow

Team Arrow Racing Association is a volunteer organisation that designs, develops and races world-class solar powered vehicles. This repository is part of our endevour to build in the open and make our learnings accessible to anyone.

You can find our more about Team Arrow on [our website](https://www.teamarrow.com.au/).
