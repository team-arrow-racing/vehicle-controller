//! Vehicle lighting as per BWSC2023 2.24

#![allow(dead_code)]

use crate::app::monotonics::MonoTimer as monotonic;
use solar_car::com::lighting::LampsState;
use stm32l4xx_hal::{
    gpio::{ErasedPin, Output, PushPull},
    prelude::PinState,
};

type OutputPin = ErasedPin<Output<PushPull>>;

pub struct Lamps {
    state: LampsState,
    on_cycle: bool,
    left_pin: OutputPin,
    right_pin: OutputPin,
    day_pin: OutputPin,
    brake_pin: OutputPin,
}

impl Lamps {
    /// Creat a new state machine instance
    pub fn new(
        left_pin: OutputPin,
        right_pin: OutputPin,
        day_pin: OutputPin,
        brake_pin: OutputPin,
    ) -> Self {
        Self {
            state: LampsState::default(),
            on_cycle: false,
            left_pin,
            right_pin,
            day_pin,
            brake_pin,
        }
    }

    // Set the state of a specific lamp
    pub fn set_lamp_state(&mut self, lamp: LampsState, state: bool) {
        match lamp {
            LampsState::INDICATOR_LEFT | LampsState::INDICATOR_RIGHT | 
                LampsState::HAZARD | LampsState::DAYTIME | LampsState::STOP => {
                self.state.set(lamp, state);
            },
            _ => {
                self.state = LampsState::empty();
            },
        }
    }


    pub fn set_state(&mut self, state: LampsState) {
        // self.state = LampsState { bits: state };
        self.state = LampsState::from(state);

        match self.state {
            LampsState::INDICATOR_LEFT => defmt::debug!("Indicating LEFT!"),
            LampsState::INDICATOR_RIGHT => defmt::debug!("Indicating RIGHT!"),
            LampsState::HAZARD => defmt::debug!("Hazard Ligghts on"),
            LampsState::DAYTIME => defmt::debug!("Daylights on"),
            LampsState::STOP => defmt::debug!("Braking!!"),
            _ => defmt::debug!("WAIT WAIT WAIT"),
        }
    }

    /// Runner that must be called regularly to keep hardware up to date
    pub fn run(&mut self) -> LampsState {
        let time = monotonic::now();

        // 50% duty cycle waveform
        let on: bool = (time.duration_since_epoch().to_millis() % 1000) > 500;

        self.left_pin.set_state(PinState::from(
            self.state.contains(LampsState::INDICATOR_LEFT) && on,
        ));
        self.right_pin.set_state(PinState::from(
            self.state.contains(LampsState::INDICATOR_RIGHT) && on,
        ));
        self.day_pin.set_state(PinState::from(
            self.state.contains(LampsState::DAYTIME),
        ));
        self.brake_pin
            .set_state(PinState::from(self.state.contains(LampsState::STOP)));

        self.state
    }
}
