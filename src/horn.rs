//! Audible warning device required by BWSC2023 2.25

#![allow(dead_code)]

use crate::app::monotonics::MonoTimer as monotonic;
use crate::app::{Duration, Instant};
use stm32l4xx_hal::{
    gpio::{ErasedPin, Output, PushPull},
    prelude::PinState,
};

type OutputPin = ErasedPin<Output<PushPull>>;

pub struct Horn {
    state: bool,
    start: Option<Instant>,
    pin: OutputPin,
}

/// Message format identifier
#[repr(u8)]
pub enum HornMessageFormat {
    Enable = 0xEE,
}

static MAXIMUM_DURATION: Duration = Duration::millis(2000);

impl Horn {
    pub fn new(pin: OutputPin) -> Self {
        Horn {
            state: false,
            start: Some(Instant::from_ticks(0)),
            pin,
        }
    }

    /// Start the horn.
    pub fn start(&mut self) -> Result<(), &'static str> {
        if !self.state {
            // do we actually want to start?
            match self.start {
                None => {
                    self.start = Some(monotonic::now());
                    self.state = true;
                    Ok(())
                }
                _ => Err("awaiting timeout"), //
            }
        } else {
            Err("already started")
        }
    }

    /// Stop the horn.
    pub fn stop(&mut self) {
        self.state = false;
    }

    pub fn set_on(&mut self) {
        self.state = true;
    }

    /// Run to update state.
    pub fn run(&mut self) {
        let time = monotonic::now();
        if self.state {
            if let Some(start) = self.start {
                if time.checked_duration_since(start).unwrap()
                    > MAXIMUM_DURATION
                {
                    self.stop();
                }
            }
        }

        self.pin.set_state(PinState::from(self.state));
    }
}
