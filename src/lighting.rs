//! Vehicle lighting as per BWSC2023 2.24

#![allow(dead_code)]

use bitflags::bitflags;

bitflags! {
    /// As per
    pub struct LampState: u8 {
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
