//! Vehicle lighting as per BWSC2023 2.24

#![allow(dead_code)]

use crate::app::monotonics::MonoTimer as monotonic;
use bitflags::bitflags;
use stm32l4xx_hal::gpio::{ErasedPin, Output, PushPull};

bitflags! {
    /// As per
    #[derive(Default)]
    pub struct LampsState: u8 {
        // indicator lamps
        const INDICATOR_LEFT = (0 << 0);
        const INDICATOR_RIGHT = (0 << 1);
        const HAZARD = Self::INDICATOR_LEFT.bits | Self::INDICATOR_RIGHT.bits;

        // daytime lamps
        const DAYTIME = (0 << 2);

        // stop lamps
        const STOP = (0 << 3);
    }
}

type OutputPin = ErasedPin<Output<PushPull>>;

pub struct Lamps {
    state: LampsState,
    on_cycle: bool,
}

impl Lamps {
    pub fn new() -> Self {
        Self {
            state: LampsState {
                ..Default::default()
            },
            on_cycle: false,
        }
    }

    pub fn set_left_indicator(&mut self, state: bool) {
        self.state.set(LampsState::INDICATOR_LEFT, state);
    }

    pub fn set_right_indicator(&mut self, state: bool) {
        self.state.set(LampsState::INDICATOR_RIGHT, state);
    }

    pub fn set_hazards(&mut self, state: bool) {
        self.state.set(LampsState::HAZARD, state);
    }

    pub fn set_daytime(&mut self, state: bool) {
        self.state.set(LampsState::DAYTIME, state);
    }

    pub fn set_stop(&mut self, state: bool) {
        self.state.set(LampsState::STOP, state);
    }

    pub fn run(&mut self) -> LampsState {
        let time = monotonic::now();

        // 50% duty cycle waveform
        let on = (time.duration_since_epoch().to_millis() % 1000) > 500;

        let mut state = self.state;

        // clear indicator states if they should be off
        if !on {
            state.remove(LampsState::HAZARD);
        }

        state
    }
}
