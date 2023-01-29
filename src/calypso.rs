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

    pub fn command_start(&mut self) -> Result<(), &'static str> {
        self.write_command("start");

        Ok(())
    }

    pub fn command_stop(&mut self) -> Result<(), &'static str> {
        self.write_command("stop");

        Ok(())
    }

    pub fn command_test(&mut self) -> Result<(), &'static str> {
        self.write_command("test");

        Ok(())
    }

    pub fn command_reboot(&mut self) -> Result<(), &'static str> {
        self.write_command("reboot");

        Ok(())
    }

    pub fn command_factory_reset(&mut self) -> Result<(), &'static str> {
        self.write_command("factoryreset");

        Ok(())
    }

    pub fn write_command(&mut self, command: &str) -> Result<&'static str, &'static str> {
        self.serial.write_str("AT+").unwrap();
        self.serial.write_str(command).unwrap();
        self.serial.write_str("\r\n").unwrap();
        
        Ok("")
    }
}
