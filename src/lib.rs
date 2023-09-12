#![deny(unsafe_code)]
#![cfg_attr(not(test), no_std)]

const MS5611_RESET: u8 = 0b0001_1110;
const MS5611_PROM_READ: u8 = 0b1010_0110;
const MS5611_READ_ADC: u8 = 0b0000_0000;

#[derive(Debug, Clone, Copy)]
pub enum OversampleRatio {
    Osr256 = 0x40,
    Osr512 = 0x42,
    Osr1024 = 0x44,
    Osr2048 = 0x46,
    Osr4096 = 0x48,
}

use embedded_hal::i2c::blocking::I2c;

struct Prom {
    /// From datasheet, C1.
    pub pressure_sensitivity: u16,
    /// From datasheet, C2.
    pub pressure_offset: u16,
    /// From datasheet, C3.
    pub temp_coef_pressure_sensitivity: u16,
    /// From datasheet, C4.
    pub temp_coef_pressure_offset: u16,
    /// From datasheet, C5.
    pub temp_ref: u16,
    /// From datasheet, C6.
    pub temp_coef_temp: u16,
}

pub struct Ms5611<I2C> {
    address: u8,
    i2c: I2C,
    prom: Option<Prom>,
}

impl<I2C: I2c> Ms5611<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Ms5611 {
            address,
            i2c,
            prom: None,
        }
    }

    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.i2c.write(self.address, &[MS5611_RESET])
    }

    pub fn read_prom(&mut self) -> Result<(), I2C::Error> {
        let mut buf: [u8; 2] = [0u8; 2];
        let mut prom = Prom {
            pressure_sensitivity: 0,
            pressure_offset: 0,
            temp_coef_pressure_sensitivity: 0,
            temp_coef_pressure_offset: 0,
            temp_ref: 0,
            temp_coef_temp: 0,
        };

        self.i2c.write(self.address, &[MS5611_PROM_READ + 2])?;
        self.i2c.read(self.address, &mut buf)?;
        prom.pressure_sensitivity = u16::from_be_bytes(buf);

        self.i2c.write(self.address, &[MS5611_PROM_READ + 4])?;
        self.i2c.read(self.address, &mut buf)?;
        prom.pressure_offset = u16::from_be_bytes(buf);

        self.i2c.write(self.address, &[MS5611_PROM_READ + 6])?;
        self.i2c.read(self.address, &mut buf)?;
        prom.temp_coef_pressure_sensitivity = u16::from_be_bytes(buf);

        self.i2c.write(self.address, &[MS5611_PROM_READ + 8])?;
        self.i2c.read(self.address, &mut buf)?;
        prom.temp_coef_pressure_offset = u16::from_be_bytes(buf);

        self.i2c.write(self.address, &[MS5611_PROM_READ + 10])?;
        self.i2c.read(self.address, &mut buf)?;
        prom.temp_ref = u16::from_be_bytes(buf);

        self.i2c.write(self.address, &[MS5611_PROM_READ + 12])?;
        self.i2c.read(self.address, &mut buf)?;
        prom.temp_coef_temp = u16::from_be_bytes(buf);

        self.prom = Some(prom);

        Ok(())
    }

    pub fn read_pressure(&mut self, osr: OversampleRatio) -> Result<u32, I2C::Error> {
        if self.prom.is_none() {
            return Ok(0_u32);
        }

        let mut data = [0u8; 4];

        self.i2c.write(self.address, &[osr as u8])?;
        self.i2c.write(self.address, &[MS5611_READ_ADC])?;
        self.i2c.read(self.address, &mut data[1..4])?;
        let d1 = i32::from_be_bytes(data);

        self.i2c.write(self.address, &[(osr as u8) + 0x10])?;
        self.i2c.write(self.address, &[MS5611_READ_ADC])?;
        self.i2c.read(self.address, &mut data[1..4])?;
        let d2 = i32::from_be_bytes(data) as i64;

        let prom = self.prom.as_ref().unwrap();

        let dt = d2 - ((prom.temp_ref as i64) << 8);
        let temperature = 2000 + (((dt * (prom.temp_coef_temp as i64)) >> 23) as i32);
        let mut offset = ((prom.pressure_offset as i64) << 16)
            + (((prom.temp_coef_pressure_offset as i64) * dt) >> 7);
        let mut sens = ((prom.pressure_sensitivity as i64) << 15)
            + (((prom.temp_coef_pressure_sensitivity as i64) * dt) >> 8);

        let mut off2 = 0;
        let mut sens2 = 0;

        // Low temperature (< 20C)
        if temperature < 2000 {
            off2 = ((5 * (temperature - 2000).pow(2)) >> 1) as i64;
            sens2 = off2 >> 1;
        }

        // Very low temperature (< -15)
        if temperature < -1500 {
            off2 += 7 * (temperature as i64 + 1500).pow(2);
            sens2 += ((11 * (temperature as i64 + 1500).pow(2)) >> 1) as i64;
        }

        offset -= off2;
        sens -= sens2;

        // Units: mbar * 100
        let pressure: i32 = (((((d1 as i64) * sens) >> 21) - offset) >> 15) as i32;

        Ok(pressure as u32)
    }
}