use core::fmt::Write;
use embedded_hal::serial::Read;

pub struct Calypso<S> {
    serial: S,
}

type Duration = systick_monotonic::fugit::SecsDurationU32;

impl<S> Calypso<S>
where
    S: Read<u8> + Write,
{
    pub fn new(serial: S) -> Self {
        Calypso { serial }
    }

    pub fn start(&mut self) -> Result<(), &'static str> {
        self.command("start");

        Ok(())
    }

    pub fn stop(&mut self) -> Result<(), &'static str> {
        self.command("stop");

        Ok(())
    }

    pub fn test(&mut self) -> Result<(), &'static str> {
        self.command("test");

        Ok(())
    }

    pub fn reboot(&mut self) -> Result<(), &'static str> {
        self.command("reboot");

        Ok(())
    }

    pub fn factory_reset(&mut self) -> Result<(), &'static str> {
        self.command("factoryreset");

        Ok(())
    }

    pub fn sleep(&mut self, time: Duration) -> Result<(), &'static str> {
        assert!(time <= Duration::secs(86400));

        self.command(write!(data));

        Ok(())
    }

    pub fn command(&mut self, command: &str) -> Result<&'static str, &'static str> {
        self.serial.write_str("AT+").unwrap();
        self.serial.write_str(command).unwrap();
        self.serial.write_str("\r\n").unwrap();
        
        Ok("")
    }
}
