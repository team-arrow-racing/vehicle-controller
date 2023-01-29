use core::fmt::Write;
use embedded_hal::serial::Read;

pub struct Calypso<S> {
    serial: S,
}

impl<S> Calypso<S>
where
    S: Read<u8> + Write,
{
    pub fn new(serial: S) -> Self {
        Calypso { serial }
    }

    pub fn command_start(&mut self) {
        self.write_command("start");
    }

    pub fn command_stop(&mut self) {
        self.write_command("stop");
    }

    pub fn command_test(&mut self) {
        self.write_command("test");
    }

    pub fn command_reboot(&mut self) {
        self.write_command("reboot");
    }

    pub fn command_factory_reset(&mut self) {
        self.write_command("factoryreset");
    }

    pub fn write_command(&mut self, command: &str) {
        self.serial.write_str("AT+").unwrap();
        self.serial.write_str(command).unwrap();
        self.serial.write_str("\r\n").unwrap();
    }
}
