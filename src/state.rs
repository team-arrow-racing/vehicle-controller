//! Vehicle state machine.

#![allow(dead_code)]

pub enum State {
    /// Off
    Safe,
    /// Telemetry, LV-bus
    Idle,
    /// Telemetry, LV-bus, HV-bus, PV-array
    Collecting,
    /// Telemetry, LV-bus, HV-bus, PV-array, Drive-train
    Driving,
}

impl State {
    /// Create a new state machine instance.
    pub fn new() -> Self {
        Self::Idle
    }

    pub fn state(self) -> State {
        self
    }
}
