use crate::error::Error;
use crate::{FtInner, PinUse};
use ftdi_mpsse::{ClockData, ClockDataOut, MpsseCmdBuilder, MpsseCmdExecutor};
use std::sync::{Arc, Mutex, MutexGuard};

/// FTDI SPI polarity.
///
/// This is a helper type to support multiple embedded-hal versions simultaneously.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FTDIPolarity {
    /// MPSSE command used to clock data in and out simultaneously.
    ///
    /// This is set by [`Spi::set_clock_polarity`].
    clk: ClockData,
    /// MPSSE command used to clock data out.
    ///
    /// This is set by [`Spi::set_clock_polarity`].
    clk_out: ClockDataOut,
}

impl From<eh1::spi::Polarity> for FTDIPolarity {
    fn from(mode: eh1::spi::Polarity) -> Self {
        match mode {
            eh1::spi::Polarity::IdleLow => FTDIPolarity {
                clk: ClockData::MsbPosIn,
                clk_out: ClockDataOut::MsbNeg,
            },
            eh1::spi::Polarity::IdleHigh => FTDIPolarity {
                clk: ClockData::MsbNegIn,
                clk_out: ClockDataOut::MsbPos,
            },
        }
    }
}

impl Default for FTDIPolarity {
    fn default() -> Self {
        Self {
            clk: ClockData::MsbPosIn,
            clk_out: ClockDataOut::MsbNeg,
        }
    }
}

/// FTDI SPI bus.
///
/// In embedded-hal version 1 this represents an exclusive SPI bus.
///
/// This is created by calling [`FtHal::spi`].
///
/// [`FtHal::spi`]: crate::FtHal::spi
#[derive(Debug)]
pub struct Spi<Device: MpsseCmdExecutor> {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtInner<Device>>>,
    /// SPI polarity
    pol: FTDIPolarity,
}

impl<Device, E> Spi<Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    pub(crate) fn new(
        mtx: Arc<Mutex<FtInner<Device>>>,
        mode: eh1::spi::Mode,
    ) -> Result<Spi<Device>, Error<E>> {
        {
            let mut lock = mtx.lock().expect("Failed to aquire FTDI mutex");
            lock.allocate_pin(0, PinUse::Spi);
            lock.allocate_pin(1, PinUse::Spi);
            lock.allocate_pin(2, PinUse::Spi);

            // clear direction of first 3 pins
            lock.direction &= !0x07;
            // set SCK (AD0) and MOSI (AD1) as output pins
            lock.direction |= 0x03;

            // set GPIO pins to new state
            let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
                .set_gpio_lower(lock.value, lock.direction)
                .send_immediate();
            lock.ft.send(cmd.as_slice())?;

            match mode.phase {
                eh1::spi::Phase::CaptureOnSecondTransition => {
                    lock.ft.send(
                        MpsseCmdBuilder::new()
                            .enable_3phase_data_clocking()
                            .send_immediate()
                            .as_slice(),
                    )?;
                }
                eh1::spi::Phase::CaptureOnFirstTransition => {
                    lock.ft.send(
                        MpsseCmdBuilder::new()
                            .disable_3phase_data_clocking()
                            .send_immediate()
                            .as_slice(),
                    )?;
                }
            }
        }

        Ok(Spi {
            mtx,
            pol: mode.polarity.into(),
        })
    }
}

impl<E> eh1::spi::Error for Error<E>
where
    E: std::error::Error,
    Error<E>: From<E>,
{
    fn kind(&self) -> eh1::spi::ErrorKind {
        eh1::spi::ErrorKind::Other
    }
}

impl<Device, E> eh1::spi::ErrorType for Spi<Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    type Error = Error<E>;
}

impl<Device, E> eh1::spi::SpiBus<u8> for Spi<Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        let data_out: Vec<u8> = vec![0; words.len()];
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data(self.pol.clk, &data_out)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.ft.send(cmd.as_slice())?;
        lock.ft.recv(words)?;

        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Error<E>> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data_out(self.pol.clk_out, words)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.ft.send(cmd.as_slice())?;

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Error<E>> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data(self.pol.clk, words)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");

        lock.ft.send(cmd.as_slice())?;
        lock.ft.recv(words)?;

        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Error<E>> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data(self.pol.clk, write)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        lock.ft.send(cmd.as_slice())?;
        lock.ft.recv(read)?;

        let remain: usize = write.len().saturating_sub(read.len());
        if remain != 0 {
            let mut remain_buf: Vec<u8> = vec![0; remain];
            lock.ft.recv(&mut remain_buf)?;
        }

        Ok(())
    }
}

impl<Device, E> ehnb1::spi::FullDuplex<u8> for Spi<Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    fn read(&mut self) -> nb::Result<u8, Error<E>> {
        let mut buf: [u8; 1] = [0];
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data(self.pol.clk, &buf)
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        match lock.ft.xfer(cmd.as_slice(), &mut buf) {
            Ok(()) => Ok(buf[0]),
            Err(e) => Err(nb::Error::Other(Error::from(e))),
        }
    }

    fn write(&mut self, byte: u8) -> nb::Result<(), Error<E>> {
        let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
            .clock_data_out(self.pol.clk_out, &[byte])
            .send_immediate();

        let mut lock = self.mtx.lock().expect("Failed to aquire FTDI mutex");
        match lock.ft.send(cmd.as_slice()) {
            Ok(()) => Ok(()),
            Err(e) => Err(nb::Error::Other(Error::from(e))),
        }
    }
}

pub struct SpiDeviceBus<'a, Device: MpsseCmdExecutor> {
    lock: MutexGuard<'a, FtInner<Device>>,
    pol: FTDIPolarity,
}

impl<Device, E> eh1::spi::ErrorType for SpiDeviceBus<'_, Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    type Error = Error<E>;
}

impl<Device, E> eh1::spi::SpiBus<u8> for SpiDeviceBus<'_, Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.lock.ft.xfer(
            MpsseCmdBuilder::new()
                .clock_data(self.pol.clk, words)
                .send_immediate()
                .as_slice(),
            words,
        )?;
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        self.lock.ft.send(
            MpsseCmdBuilder::new()
                .clock_data_out(self.pol.clk_out, words)
                .send_immediate()
                .as_slice(),
        )?;
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        self.lock.ft.xfer(
            MpsseCmdBuilder::new()
                .clock_data(self.pol.clk, write)
                .send_immediate()
                .as_slice(),
            read,
        )?;
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        self.lock.ft.xfer(
            MpsseCmdBuilder::new()
                .clock_data(self.pol.clk, words)
                .send_immediate()
                .as_slice(),
            words,
        )?;
        Ok(())
    }
}

/// FTDI SPI device, a SPI bus with chip select pin.
///
/// This is created by calling [`FtHal::spi_device`].
///
/// This is specific to embedded-hal version 1.
///
/// [`FtHal::spi_device`]: crate::FtHal::spi_device
#[derive(Debug)]
pub struct SpiDevice<Device: MpsseCmdExecutor> {
    /// Parent FTDI device.
    mtx: Arc<Mutex<FtInner<Device>>>,
    /// SPI polarity
    pol: FTDIPolarity,
    /// Chip select pin index.  0-7 for the FT232H.
    cs_idx: u8,
    /// The embedded_hal SPI mode value.
    mode: eh1::spi::Mode,
}

impl<Device, E> SpiDevice<Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    pub(crate) fn new(
        mtx: Arc<Mutex<FtInner<Device>>>,
        cs_idx: u8,
        mode: eh1::spi::Mode,
    ) -> Result<SpiDevice<Device>, Error<E>> {
        {
            let mut lock = mtx.lock().expect("Failed to aquire FTDI mutex");
            lock.allocate_pin(0, PinUse::Spi);
            lock.allocate_pin(1, PinUse::Spi);
            lock.allocate_pin(2, PinUse::Spi);
            lock.allocate_pin(cs_idx, PinUse::Output);

            let cs_mask: u8 = 1 << cs_idx;

            // clear direction of first 3 pins and CS
            lock.direction &= !(0x07 | cs_mask);
            // set SCK (AD0) and MOSI (AD1), and CS as output pins
            lock.direction |= 0x03 | cs_mask;
            // Ensure CS is not asserted.
            lock.value |= cs_mask;

            match mode.phase {
                eh1::spi::Phase::CaptureOnSecondTransition => {
                    lock.ft.send(
                        MpsseCmdBuilder::new()
                            .enable_3phase_data_clocking()
                            .send_immediate()
                            .as_slice(),
                    )?;
                }
                eh1::spi::Phase::CaptureOnFirstTransition => {
                    lock.ft.send(
                        MpsseCmdBuilder::new()
                            .disable_3phase_data_clocking()
                            .send_immediate()
                            .as_slice(),
                    )?;
                }
            }

            // set GPIO pins to new state
            let cmd: MpsseCmdBuilder = MpsseCmdBuilder::new()
                .set_gpio_lower(lock.value, lock.direction)
                .send_immediate();
            lock.ft.send(cmd.as_slice())?;
        }

        Ok(Self {
            mtx,
            pol: mode.polarity.into(),
            cs_idx,
            mode,
        })
    }

    pub(crate) fn cs_mask(&self) -> u8 {
        1 << self.cs_idx
    }
}

impl<Device, E> eh1::spi::ErrorType for SpiDevice<Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    type Error = Error<E>;
}

impl<'a, Device, E> eh1::spi::SpiDevice for SpiDevice<Device>
where
    Device: MpsseCmdExecutor<Error = E>,
    E: std::error::Error,
    Error<E>: From<E>,
{
    fn transaction(
        &mut self,
        operations: &mut [eh1::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        // lock the bus
        let mut lock: MutexGuard<FtInner<Device>> =
            self.mtx.lock().expect("Failed to aquire FTDI mutex");
        let direction: u8 = lock.direction;

        let sck_idle_bit = match self.mode.polarity {
            eh1::spi::Polarity::IdleLow => 0x00,
            eh1::spi::Polarity::IdleHigh => 0x01,
        };

        // Ensure SCLK is set to the value it idles at by clearing it then OR.
        lock.value = (lock.value & !0x01) | sck_idle_bit;
        let value = lock.value;
        lock.ft.send(
            MpsseCmdBuilder::new()
                .set_gpio_lower(value, direction)
                .send_immediate()
                .as_slice(),
        )?;

        // Turn CS off.
        lock.value = lock.value & !self.cs_mask();
        let value = lock.value;
        lock.ft.send(
            MpsseCmdBuilder::new()
                .set_gpio_lower(value, direction)
                .send_immediate()
                .as_slice(),
        )?;

        let mut bus: SpiDeviceBus<Device> = SpiDeviceBus {
            lock,
            pol: self.pol,
        };

        for op in operations {
            match op {
                eh1::spi::Operation::Read(buffer) => {
                    eh1::spi::SpiBus::read(&mut bus, buffer)?;
                }
                eh1::spi::Operation::Write(buffer) => {
                    eh1::spi::SpiBus::write(&mut bus, buffer)?;
                }
                eh1::spi::Operation::Transfer(read, write) => {
                    eh1::spi::SpiBus::transfer(&mut bus, read, write)?;
                }
                eh1::spi::Operation::TransferInPlace(buffer) => {
                    eh1::spi::SpiBus::transfer_in_place(&mut bus, buffer)?;
                }
                eh1::spi::Operation::DelayNs(micros) => {
                    std::thread::sleep(std::time::Duration::from_nanos((*micros).into()));
                }
            }
        }

        // flush the bus
        {
            use eh1::spi::SpiBus;
            bus.flush()?;
        }

        let mut lock: MutexGuard<FtInner<Device>> = bus.lock;

        lock.value = lock.value | self.cs_mask();
        let value = lock.value;
        lock.ft.send(
            MpsseCmdBuilder::new()
                .set_gpio_lower(value, direction)
                .send_immediate()
                .as_slice(),
        )?;

        // unlocking the bus is implicit via Drop
        Ok(())
    }
}
