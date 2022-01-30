//! This prerenders the data, so that no calculations have to be performed while sending the data.
//!
//! This approach minimizes timing issues, at the cost of much higher ram usage.
//! It also increases the needed time.

use embedded_hal as hal;

use hal::spi::{FullDuplex, Mode, Phase, Polarity};

use smart_leds_trait::{RGB, RGB8};

use nb;
use nb::block;

/// SPI mode that can be used for this crate
///
/// Provided for convenience
/// Doesn't really matter
pub const MODE: Mode = Mode {
    polarity: Polarity::IdleLow,
    phase: Phase::CaptureOnFirstTransition,
};

pub struct Ws2812<SPI, const N: usize>
where
    [u8; N * 3]: Sized,
{
    spi: SPI,
    data: [u8; N * 3],
}

impl<SPI, E, const N: usize> Ws2812<SPI, N>
where
    SPI: FullDuplex<u8, Error = E>,
    [u8; N * 3]: Sized,
{
    // Byte amount to represent a color. Must be same value as array size factor.
    const CHANNEL_AMOUNT: usize = 3;

    /// Use ws2812 devices via spi
    ///
    /// The SPI bus should run within 2 MHz to 3.8 MHz
    ///
    /// You may need to look at the datasheet and your own hal to verify this.
    ///
    /// You need to provide a buffer `data`, whose length is at least 12 * the
    /// length of the led strip + 20 byes (or 40, if using the `mosi_idle_high` feature)
    ///
    /// Please ensure that the mcu is pretty fast, otherwise weird timing
    /// issues will occur
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            data: [0; N * 3],
        }
    }
}

impl<SPI, E, const N: usize> Ws2812<SPI, N>
where
    SPI: FullDuplex<u8, Error = E>,
    [u8; N * 3]: Sized,
{
    pub fn value_at(&self, index: usize) -> u8 {
        self.data[index]
    }

    pub fn led_color(&self, index: usize) -> RGB8 {
        let offset = index * 3;
        RGB::new(
            self.data[offset + 1],
            self.data[offset],
            self.data[offset + 2],
        )
    }

    pub fn set_led_color(&mut self, index: usize, color: RGB8) {
        let offset = index * Self::CHANNEL_AMOUNT;
        self.data[offset] = color.g;
        self.data[offset + 1] = color.r;
        self.data[offset + 2] = color.b;
    }
}

impl<SPI, E, const N: usize> Ws2812<SPI, N>
where
    SPI: FullDuplex<u8, Error = E>,
    [u8; N * 3]: Sized,
{
    /// Write a single byte for ws2812 devices to spi
    fn send_byte(&mut self, mut value: u8) -> Result<(), E> {
        // Send two bits in one spi byte. High time first, then the low time
        // The maximum for T0H is 500ns, the minimum for one bit 1063 ns.
        // These result in the upper and lower spi frequency limits
        let patterns = [0b1000_1000, 0b1000_1110, 0b11101000, 0b11101110];
        for _ in 0..4 {
            let bits = (value & 0b1100_0000) >> 6;
            block!(self.spi.send(patterns[bits as usize]))?;
            value <<= 2;
        }

        Ok(())
    }

    /// Send the pre rendered data to the LEDs.
    pub fn send_data(&mut self) -> Result<(), E> {
        // We introduce an offset in the FIFO here, so there's always one byte in transit
        // Some MCUs (like the stm32f1) only a one byte FIFO, which would result
        // in overrun error if two bytes need to be stored
        block!(self.spi.send(0))?;
        if cfg!(feature = "mosi_idle_high") {
            for _ in 0..140 {
                block!(self.spi.send(0))?;
                block!(self.spi.read())?;
            }
        }
        for b in self.data {
            self.send_byte(b)?;
            block!(self.spi.read())?;
        }
        for _ in 0..140 {
            block!(self.spi.send(0))?;
            block!(self.spi.read())?;
        }
        // Now, resolve the offset we introduced at the beginning
        block!(self.spi.read())?;
        Ok(())
    }
}
