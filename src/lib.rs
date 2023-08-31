#![deny(unsafe_code)]
#![cfg_attr(not(test), no_std)]

const MS5611_RESET: u8 = 0b0001_1110;
const MS5611_PROM_READ: u8 = 0b1010_0110;
const MS5611_READ_ADC: u8 = 0b0000_0000;

pub enum OversampleRatio {
    Osr256 = 0x40,
    Osr512 = 0x42,
    Osr1024 = 0x44,
    Osr2048 = 0x46,
    Osr4096 = 0x48,
}

use embedded_hal::i2c::I2c;
pub struct Ms5611<I2C> {
    address: u8,
    i2c: I2C,
}

impl<I2C: I2c> Ms5611<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Ms5611 { address, i2c }
    }

    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[MS5611_RESET])
    }

    pub fn prom_read(&mut self, prom: &mut [u8; 2]) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[MS5611_PROM_READ])?;
        self.i2c.read(self.address, prom)
    }

    pub fn read_pressure(&mut self, osr: OversampleRatio) -> Result<u32, I2C::Error> {
        self.i2c.write(self.address, &[osr as u8])?;
        self.i2c.write(self.address, &[MS5611_READ_ADC])?;
        let mut data = [0u8; 3];
        self.i2c.read(self.address, &mut data)?;

        Ok((u32::from(data[0]) << 16) | (u32::from(data[1]) << 8) | u32::from(data[2]))
    }

    pub fn read_temperature(&mut self, osr: OversampleRatio) -> Result<u32, I2C::Error> {
        let cmd = osr as u8 + 0x10;

        self.i2c.write(self.address, &[cmd])?;
        self.i2c.write(self.address, &[MS5611_READ_ADC])?;
        let mut data = [0u8; 3];
        self.i2c.read(self.address, &mut data)?;

        Ok((u32::from(data[0]) << 16) | (u32::from(data[1]) << 8) | u32::from(data[2]))
    }
}
