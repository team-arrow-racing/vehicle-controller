//! Vehicle lighting as per BWSC2023 2.24

#![allow(dead_code)]

use crate::app::monotonics::MonoTimer as monotonic;
use bitflags::bitflags;
use stm32l4xx_hal::{
    gpio::{ErasedPin, Output, PushPull},
    prelude::PinState,
};

type OutputPin = ErasedPin<Output<PushPull>>;

bitflags! {
    /// As per
    #[derive(Default)]
    pub struct LampsState: u8 {
        // indicator lamps
        const INDICATOR_LEFT = 1 << 0;
        const INDICATOR_RIGHT = 1 << 1;
        const HAZARD = Self::INDICATOR_LEFT.bits | Self::INDICATOR_RIGHT.bits;

        // daytime lamps
        const DAYTIME = 1 << 2;

        // stop lamps
        const STOP = 1 << 3;
    }
}

pub struct Lamps {
    state: LampsState,
    on_cycle: bool,
    left_pin: OutputPin,
    right_pin: OutputPin,
    day_pin: OutputPin,
    brake_pin: OutputPin
}

impl Lamps {
    /// Creat a new state machine instance
    pub fn new(left_pin: OutputPin,
                right_pin: OutputPin,
                day_pin: OutputPin,
                brake_pin: OutputPin) -> Self {
        Self {
            state: LampsState {
                ..Default::default()
            },
            on_cycle: false,
            left_pin,
            right_pin,
            day_pin,
            brake_pin
        }
    }

    /// Set left indicator state
    pub fn set_left_indicator(&mut self, state: bool) {
        self.state.set(LampsState::INDICATOR_LEFT, state);
    }

    /// Set right indicator state
    pub fn set_right_indicator(&mut self, state: bool) {
        self.state.set(LampsState::INDICATOR_RIGHT, state);
    }

    /// Set hazard lights state
    pub fn set_hazards(&mut self, state: bool) {
        self.state.set(LampsState::HAZARD, state);
    }

    /// Set the daytime running lights state
    pub fn set_daytime(&mut self, state: bool) {
        self.state.set(LampsState::DAYTIME, state);
    }

    /// Set the stop light state
    pub fn set_stop(&mut self, state: bool) {
        self.state.set(LampsState::STOP, state);
    }

    /// Turn all of the lights off
    pub fn all_off(&mut self) {
        self.state = LampsState::empty();
    }

    /// Runner that must be called regularly to keep hardware up to date
    pub fn run(&mut self) -> LampsState {
        let time = monotonic::now();

        // 50% duty cycle waveform
        let on = (time.duration_since_epoch().to_millis() % 1000) > 500;

        let mut state = self.state;

        // clear indicator states if they should be off
        if !on {
            state.remove(LampsState::HAZARD);
        }

        self.left_pin.set_state(PinState::from(state.contains(LampsState::INDICATOR_LEFT)));
        self.right_pin.set_state(PinState::from(state.contains(LampsState::INDICATOR_RIGHT)));
        self.day_pin.set_state(PinState::from(state.contains(LampsState::DAYTIME)));
        self.brake_pin.set_state(PinState::from(state.contains(LampsState::STOP)));

        state
    }
}
