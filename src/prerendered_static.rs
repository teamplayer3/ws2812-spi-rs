//! This prerenders the data, so that no calculations have to be performed while sending the data.
//!
//! This approach minimizes timing issues, at the cost of much higher ram usage.
//! It also increases the needed time.

use embedded_hal as hal;

use hal::spi::{FullDuplex, Mode, Phase, Polarity};

use smart_leds_trait::{SmartLedsWrite, RGB, RGB8};

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
    index: usize,
}

impl<SPI, E, const N: usize> Ws2812<SPI, N>
where
    SPI: FullDuplex<u8, Error = E>,
    [u8; N * 3]: Sized,
{
    /// Use ws2812 devices via spi
    ///
    /// The SPI bus should run within 2 MHz to 3.8 MHz
    ///
    /// You may need to look at the datasheet and your own hal to verify this.
    ///
    /// You need to provide a buffer `data`, whoose length is at least 12 * the
    /// length of the led strip + 20 byes (or 40, if using the `mosi_idle_high` feature)
    ///
    /// Please ensure that the mcu is pretty fast, otherwise weird timing
    /// issues will occur
    pub fn new(spi: SPI) -> Self {
        Self {
            spi,
            data: [0; N * 3],
            index: 0,
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
            self.data[offset],
            self.data[offset + 1],
            self.data[offset + 2],
        )
    }

    pub fn set_value_at(&mut self, index: usize, value: u8) {
        self.data[index] = value;
    }

    pub fn set_led_color(&mut self, index: usize, color: RGB8) {
        let offset = index * 3;
        self.data[offset] = color.r;
        self.data[offset + 1] = color.g;
        self.data[offset + 2] = color.b;
    }
}

impl<SPI, E, const N: usize> Ws2812<SPI, N>
where
    SPI: FullDuplex<u8, Error = E>,
    [u8; N * 3]: Sized,
{
    /// Write a single byte for ws2812 devices
    fn write_byte(&mut self, mut data: u8) {
        // Send two bits in one spi byte. High time first, then the low time
        // The maximum for T0H is 500ns, the minimum for one bit 1063 ns.
        // These result in the upper and lower spi frequency limits
        let patterns = [0b1000_1000, 0b1000_1110, 0b11101000, 0b11101110];
        for _ in 0..4 {
            let bits = (data & 0b1100_0000) >> 6;
            self.data[self.index] = patterns[bits as usize];
            self.index += 1;
            data <<= 2;
        }
    }

    fn send_data(&mut self) -> Result<(), E> {
        // We introduce an offset in the fifo here, so there's always one byte in transit
        // Some MCUs (like the stm32f1) only a one byte fifo, which would result
        // in overrun error if two bytes need to be stored
        block!(self.spi.send(0))?;
        if cfg!(feature = "mosi_idle_high") {
            for _ in 0..140 {
                block!(self.spi.send(0))?;
                block!(self.spi.read())?;
            }
        }
        for b in self.data[..self.index].iter() {
            block!(self.spi.send(*b))?;
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

impl<SPI, E, const N: usize> SmartLedsWrite for Ws2812<SPI, N>
where
    SPI: FullDuplex<u8, Error = E>,
    [u8; N * 3]: Sized,
{
    type Error = E;
    type Color = RGB8;
    /// Write all the items of an iterator to a ws2812 strip
    fn write<T, I>(&mut self, iterator: T) -> Result<(), E>
    where
        T: Iterator<Item = I>,
        I: Into<Self::Color>,
    {
        self.index = 0;

        for item in iterator {
            let item = item.into();
            self.write_byte(item.g);
            self.write_byte(item.r);
            self.write_byte(item.b);
        }
        self.send_data()
    }
}
