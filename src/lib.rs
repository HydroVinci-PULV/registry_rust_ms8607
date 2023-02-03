//! ms8607.rs
//!
//! This is a library for the MS8607 Pressure, Temperature, and Humidity Sensor from TE Connectivity
//!
//! Based on the Adafruit_MS8607 library from Adafruit and the
//! [datasheet](https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS8607-02BA01&DocType=DS&DocLang=English) for the MS8607 sensor from Tyco Electronics.
//!
//! Author: Antoine Raulin, 2023

#![no_std]
#![allow(unused)]
use embedded_hal::blocking::{i2c::{Write, WriteRead}, delay::DelayMs};


/// Enum containing all possible types of errors when interacting with the sensor
#[derive(Debug)]
pub enum Error<E> {
    CommunicationError(E),
    SensorDetectFailed,
    RegisterReadFailed,
    RegisterWriteFailed,
    CrcCheckFailed,
    InvalidResolution,
}

// Allow converting the Error type from the I2C device to the error enum
impl<E> From<E> for Error<E> {
    fn from(e: E) -> Self {
        Error::CommunicationError(e)
    }
}
pub struct MS8607
{
    // calibration constants
    press_sens: u16,
    press_offset: u16,
    press_sens_temp_coeff: u16,
    press_offset_temp_coeff: u16,
    ref_temp: u16,
    temp_temp_coeff: u16,
    /// The current I2C mode to use for humidity reads
    hum_sensor_i2c_read_mode: Ms8607HumidityClockStretch,
    psensor_resolution_osr: Ms8607PressureResolution,
    /// The current pressure measurement
    _pressure: f64,
    /// The current temperature measurement
    _temperature: f64,
    /// The current humidity measurement
    _humidity: f64,
}

impl MS8607
{
    /// Create a new MS8607 instance
    pub fn new() -> Self {
        MS8607 {
            press_sens: 0,
            press_offset: 0,
            press_sens_temp_coeff: 0,
            press_offset_temp_coeff: 0,
            ref_temp: 0,
            temp_temp_coeff: 0,
            hum_sensor_i2c_read_mode: Ms8607HumidityClockStretch::Ms8607I2cHold,
            psensor_resolution_osr: Ms8607PressureResolution::Osr4096,
            _pressure: 0.0,
            _temperature: 0.0,
            _humidity: 0.0,
        }
    }

    /// Sets up the hardware and initializes I2C
    pub fn begin<I2C,Delay, E>(&mut self, i2c:&mut I2C, delay:&mut Delay) -> Result<(), Error<E>> where
    I2C: Write<Error = E> + WriteRead<Error = E>, Delay: DelayMs<u16> {
        self.reset(i2c, delay)?;
        self.init(i2c)?;
        Ok(())
    }

    /// Initializer for post i2c/spi init
    pub fn init<I2C, E>(&mut self, i2c:&mut I2C) -> Result<(), Error<E>> where
    I2C: Write<Error = E> + WriteRead<Error = E>,{
        self._fetch_temp_calibration_values(i2c)?;
        self.enable_humidity_clock_stretching(false);

        self.set_humidity_resolution(i2c, Ms8607HumidityResolution::Osr12b)?;

        self.set_pressure_resolution(Ms8607PressureResolution::Osr4096);

        Ok(())
    }

    /// Allow the MS8607 to hold the clock line low until it completes the
    /// requested measurements
    ///
    /// ## Arguments
    /// enable_stretching: true to enable clock stretching, false to disable
    pub fn enable_humidity_clock_stretching(&mut self, enable_stretching: bool) {
        if enable_stretching {
            self.hum_sensor_i2c_read_mode = Ms8607HumidityClockStretch::Ms8607I2cHold;
        } else {
            self.hum_sensor_i2c_read_mode = Ms8607HumidityClockStretch::Ms8607I2cNoHold;
        }
    }

    /// Set the resolution for humidity readings
    ///
    /// ## Arguments
    /// resolution: the resolution to set
    pub fn set_humidity_resolution<I2C, E>(
        &mut self, i2c:&mut I2C,
        resolution: Ms8607HumidityResolution,
    )  -> Result<(), Error<E>> where
    I2C: Write<Error = E> + WriteRead<Error = E>, {
        let mut reg_value = self._read_humidity_user_register(i2c)?;

        // unset current value
        reg_value &= _MS8607::HSENSOR_USER_REG_RESOLUTION_MASK;
        // set new value
        reg_value |= resolution as u8 & _MS8607::HSENSOR_USER_REG_RESOLUTION_MASK;

        self._write_humidity_user_register(i2c, reg_value)
    }

    /// Set the resolution for pressure readings
    ///
    /// ## Arguments
    /// resolution: the resolution to set
    pub fn set_pressure_resolution(&mut self, resolution: Ms8607PressureResolution) {
        self.psensor_resolution_osr = resolution;
    }

    /// Get the currently set resolution for humidity readings
    pub fn get_humidity_resolution<I2C, E>(&mut self, i2c:&mut I2C) -> Result<Ms8607HumidityResolution, Error<E>> where
    I2C: Write<Error = E> + WriteRead<Error = E>, {
        let reg_value = self._read_humidity_user_register(i2c)?;
        let resolution = reg_value & _MS8607::HSENSOR_USER_REG_RESOLUTION_MASK;
        match resolution {
            0 => Ok(Ms8607HumidityResolution::Osr12b),
            1 => Ok(Ms8607HumidityResolution::Osr8b),
            2 => Ok(Ms8607HumidityResolution::Osr10b),
            3 => Ok(Ms8607HumidityResolution::Osr11b),
            _ => Err(Error::InvalidResolution),
        }
    }

    /// Get the currently set resolution for pressure readings
    pub fn get_pressure_resolution(&self) -> Ms8607PressureResolution {
        self.psensor_resolution_osr
    }

    pub fn get_measurements<I2C,Delay, E>(&mut self, i2c:&mut I2C, delay:&mut Delay) -> Result<(f64, f64, f64), Error<E>> where
    I2C: Write<Error = E> + WriteRead<Error = E>,Delay: DelayMs<u16> {
        self._read(i2c, delay)?;
        self._read_humidity(i2c)?;
        Ok((self._pressure, self._temperature, self._humidity))
    }

    /**
    Read the current pressure and temperature
    */
    fn _read<I2C,Delay, E>(&mut self, i2c:&mut I2C, delay:&mut Delay) -> Result<(), Error<E>> where
    I2C: Write<Error = E> + WriteRead<Error = E>, Delay: DelayMs<u16> {
        let mut cmd = [0u8; 1];
        let mut buffer = [0u8; 3];

        // First read temperature
        cmd[0] = self.psensor_resolution_osr as u8 * 2;
        cmd[0] |= _MS8607::PSENSOR_START_TEMPERATURE_ADC_CONVERSION;
        i2c.write(_MS8607::PT_ADDRESS, &cmd).map_err(Error::from)?;
        
        // wait 18ms
        delay.delay_ms(20);

        cmd = [_MS8607::PSENSOR_READ_ADC];
        i2c.write_read(_MS8607::PT_ADDRESS, &cmd, &mut buffer).map_err(Error::from)?;
        
        let raw_temp = ((buffer[0] as u32) << 16) | ((buffer[1] as u32) << 8) | (buffer[2] as u32);
        delay.delay_ms(20);
        // Now read pressure
        cmd[0] = self.psensor_resolution_osr as u8 * 2;
        cmd[0] |= _MS8607::PSENSOR_START_PRESSURE_ADC_CONVERSION;
        i2c.write(_MS8607::PT_ADDRESS, &cmd).map_err(Error::from)?;
       
        // wait 18ms
        delay.delay_ms(20);
        buffer = [0u8; 3];

        cmd = [_MS8607::PSENSOR_READ_ADC];
        i2c.write_read(_MS8607::PT_ADDRESS, &cmd, &mut buffer).map_err(Error::from)?;
        
        let raw_pressure =
            ((buffer[0] as u32) << 16) | ((buffer[1] as u32) << 8) | (buffer[2] as u32);
        self._apply_ptcorrections(raw_temp as i32, raw_pressure as i32);
        Ok(())
    }

    /**
    humidity user register value: 0b10
    humidity resolution raw value: 0x0
    Temperature: 29.85 degrees C
    Pressure: 1008.94 hPa
    Relative Humidity: 25.94 %rH
    */
    fn _read_humidity<I2C, E>(&mut self, i2c:&mut I2C) -> Result<(), Error<E>> where
    I2C: Write<Error = E> + WriteRead<Error = E>, {
        let mut buffer = [0u8; 3];
        self.get_humidity_resolution(i2c)?;
        let cmd = [Ms8607HumidityClockStretch::Ms8607I2cHold as u8];
        i2c.write_read(_MS8607::HUM_ADDRESS, &cmd, &mut buffer).map_err(Error::from)?;
        
        let raw_hum: u16 = (buffer[0] as u16) << 8 | (buffer[1] as u16);
        let crc = buffer[2];
        self._hsensor_crc_check(raw_hum, crc)?;
        self._humidity = ((raw_hum as f32) * _MS8607::MS8607_RH_LSB) as f64;
        self._humidity -= 6.0;
        Ok(())
    }

    /**
    A function that applies pressure and temperature corrections to the raw sensor data

    ## Arguments
    * `raw_temp` - raw temperature data from the sensor
    * `raw_pressure` - raw pressure data from the sensor
    */
    fn _apply_ptcorrections(&mut self, raw_temp: i32, raw_pressure: i32) {
        let mut off: i64;
        let mut sens: i64;
        let t2: i64;
        let mut off2: i64;
        let mut sens2: i64;

        let d_t = raw_temp - ((self.ref_temp as i32) << 8);

        // Actual temperature = 2000 + dT * TEMPSENS
        let temp = (2000 + (((d_t as i64) * (self.temp_temp_coeff as i64)) >> 23)) as i32;

        // Second order temperature compensation
        if temp < 2000 {
            t2 = (3 * ((d_t as i64) * (d_t as i64))) >> 33;
            off2 = 61 * ((temp as i64) - 2000) * ((temp as i64) - 2000) / 16;
            sens2 = 29 * ((temp as i64) - 2000) * ((temp as i64) - 2000) / 16;

            if temp < -1500 {
                off2 += 17 * ((temp as i64) + 1500) * ((temp as i64) + 1500);
                sens2 += 9 * ((temp as i64) + 1500) * ((temp as i64) + 1500);
            }
        } else {
            t2 = (5 * ((d_t as i64) * (d_t as i64))) >> 38;
            off2 = 0;
            sens2 = 0;
        }

        // OFF = OFF_T1 + TCO * dT
        off = ((self.press_offset as i64) << 17)
            + (((self.press_offset_temp_coeff as i64) * (d_t as i64)) >> 6);
        off -= off2;

        // Sensitivity at actual temperature = SENS_T1 + TCS * dT
        sens = ((self.press_sens as i64) << 16)
            + (((self.press_sens_temp_coeff as i64) * (d_t as i64)) >> 7);
        sens -= sens2;

        // Temperature compensated pressure = D1 * SENS - OFF
        let p = ((((raw_pressure as i64) * sens) >> 21) - off) >> 15;

        self._temperature = ((temp as f64) - t2 as f64) / 100.0;
        self._pressure = p as f64 / 100.0;
    }

    /**
    Write to the humidity sensor's user register

    ## Parameters

    * `new_reg_value` - The new value to set the user register to

    ## Return

     Returns `Ok(())` if the write is successful, `Err("Failed to write to I2C")` if the I2C write fails
    */
    fn _write_humidity_user_register<I2C, E>(&mut self,i2c:&mut I2C, new_reg_value: u8) -> Result<(), Error<E>> where
    I2C: Write<Error = E> + WriteRead<Error = E>, {
        let buffer = [_MS8607::HSENSOR_WRITE_USER_REG_COMMAND, new_reg_value];
        i2c.write(_MS8607::HUM_ADDRESS, &buffer).map_err(Error::from)?;
        Ok(())
    }

    /**
    Reads the user register from the humidity sensor of the MS8607 sensor
    ## Errors
    Returns an error if it fails to write to the I2C bus or fails to read from the I2C bus

    ## Returns
    Returns the value of the user register as a `u8` if successful.
    */
    fn _read_humidity_user_register<I2C, E>(&mut self, i2c:&mut I2C) -> Result<u8, Error<E>> where
    I2C: Write<Error = E> + WriteRead<Error = E>, {
        let mut cmd = [_MS8607::HSENSOR_READ_USER_REG_COMMAND];
        let mut buffer = [0u8; 1];


        i2c.write_read(_MS8607::HUM_ADDRESS, &cmd, &mut buffer).map_err(Error::from)?;
        
        Ok(buffer[0])
    }

    /**
    Fetches the temperature calibration values from the sensor's PROM memory
    and stores them in the corresponding fields of the struct.

    ## Errors

    Returns an error if the CRC check of the retrieved data fails.
    */
    fn _fetch_temp_calibration_values<I2C, E>(&mut self, i2c:&mut I2C) -> Result<(), Error<E>>where
    I2C: Write<Error = E> + WriteRead<Error = E>, {
        let mut offset: u8 = 0;
        let mut buffer = [0u16; 8];
        let mut tmp_buffer = [0u8; 2];

        buffer
            .iter_mut()
            .enumerate()
            .take(7usize)
            .for_each(|(i, v)| {
                offset = 2 * (i as u8);
                let cmd = [_MS8607::PROM_ADDRESS_READ_ADDRESS_0 + offset];
                let req = i2c.write_read(_MS8607::PT_ADDRESS, &cmd,&mut tmp_buffer);
                if req.is_err() {
                    return;
                }
                *v = (tmp_buffer[0] as u16) << 8;
                *v |= tmp_buffer[1] as u16;
            });
        // check if every address has been read
        if offset != 12 {
            return Err(Error::RegisterReadFailed);
        }
        // let buffer_slice = &mut buffer[..];
        self._psensor_crc_check(&mut buffer)?;

        self.press_sens = buffer[1];
        self.press_offset = buffer[2];
        self.press_sens_temp_coeff = buffer[3];
        self.press_offset_temp_coeff = buffer[4];
        self.ref_temp = buffer[5];
        self.temp_temp_coeff = buffer[6];
        Ok(())
    }

    /**
    Function that performs the CRC check on the PROM data of the pressure sensor.
    It takes in the pointer to the PROM data array, and the expected CRC value as inputs.
    It returns Ok(()) if the check passes, and an error message "CRC check failed" otherwise.

    ## Arguments

    * `n_prom` - A mutable reference to an array of u16, containing the PROM data of the pressure sensor.
    * `crc` - A u8, representing the expected CRC value of the PROM data.

    # Example
    ```rust
    let mut n_prom = [0u16; 8];
    let crc = 0u8;
    let result = self._psensor_crc_check(&mut n_prom, crc);
    ```
    */
    fn _psensor_crc_check<E>(&mut self, n_prom: &mut [u16; 8]) -> Result<(), Error<E>> {
        let mut n_rem = 0;
        let crc = ((n_prom[0] & 0xF000) >> 12) as u8;
        n_prom[0] &= 0x0FFF; // Mask off CRC bits
        n_prom[7] = 0; // Subsidiary value, set to 0
        for cnt in 0..16 {
            // operation is performed on bytes
            // choose LSB or MSB
            if cnt % 2 == 1 {
                n_rem ^= (n_prom[cnt >> 1] & 0x00FF);
            } else {
                n_rem ^= n_prom[cnt >> 1] >> 8;
            }
            for n_bit in 0..8 {
                if n_rem & 0x8000 != 0 {
                    n_rem = (n_rem << 1) ^ 0x3000;
                } else {
                    n_rem <<= 1;
                }
            }
        }
        n_rem = (n_rem >> 12) & 0x000F; // final 4-bit reminder is CRC code
        if n_rem != crc as u16 {
            return Err(Error::CrcCheckFailed);
        }
        Ok(())
    }

    /**
    This function checks the validity of the given value using the Cyclic Redundancy Check (CRC) algorithm.

    ## Arguments
    * `value`: The value to check the validity of.
    * `crc`: The expected cyclic redundancy check value.

    ## Returns
    `Ok(())` if the value is valid and `Err("CRC check failed")` if the value is not valid.
    */
    fn _hsensor_crc_check<E>(&mut self, value: u16, crc: u8) -> Result<(), Error<E>> {
        let mut polynom: u32 = 0x988000; // x^8 + x^5 + x^4 + 1
        let mut msb: u32 = 0x800000;
        let mut mask: u32 = 0xFF8000;
        let mut result: u32 = (value as u32) << 8; // Pad with zeros as specified in spec

        while msb != 0x80 {
            // Check if msb of current value is 1 and apply XOR mask
            if result & msb != 0 {
                result = ((result ^ polynom) & mask) | (result & !mask);
            }

            // Shift by one
            msb >>= 1;
            mask >>= 1;
            polynom >>= 1;
        }
        if (result) as u8 != crc {
            return Err(Error::CrcCheckFailed);
        }
        Ok(())
    }

    /// Reset the sensors to their initial state
    pub fn reset<I2C,Delay, E>(&mut self, i2c:&mut I2C, delay: &mut Delay) -> Result<(), Error<E>> where
    I2C: Write<Error = E> + WriteRead<Error = E>,Delay: DelayMs<u16> {
        let cmd = [_MS8607::P_T_RESET];
        i2c.write(_MS8607::PT_ADDRESS, &cmd).map_err(Error::from)?;
        
        let cmd = [_MS8607::HSENSOR_RESET_COMMAND];
        i2c.write(_MS8607::HUM_ADDRESS, &cmd).map_err(Error::from)?;
        
        // wait 15ms
        delay.delay_ms(15);
        Ok(())
    }
}

impl Default for MS8607 {
    fn default() -> Self {
        Self::new()
    }
}

struct _MS8607 {}

impl _MS8607 {
    // I2C ADDRESS/BITS/SETTINGS. The MS8607 uses two different I2C addresses

    /// The pressure and temperature I2C address for the sensor
    pub const PT_ADDRESS: u8 = 0x76;
    /// The default pressure and temperature I2C address for the sensor
    pub const HUM_ADDRESS: u8 = 0x40;

    // HSENSOR device commands
    /// reset command
    pub const HSENSOR_RESET_COMMAND: u8 = 0xFE;
    /// read humidity w hold command
    pub const HSENSOR_READ_HUMIDITY_W_HOLD_COMMAND: u8 = 0xE5;
    /// read humidity w/o hold command
    pub const HSENSOR_READ_HUMIDITY_WO_HOLD_COMMAND: u8 = 0xF5;
    /// read serial first 8bytes command
    pub const HSENSOR_READ_SERIAL_FIRST_8BYTES_COMMAND: u16 = 0xFA0F;
    /// read serial last 6bytes command
    pub const HSENSOR_READ_SERIAL_LAST_6BYTES_COMMAND: u16 = 0xFCC9;
    /// write user reg command
    pub const HSENSOR_WRITE_USER_REG_COMMAND: u8 = 0xE6;
    /// read user reg command
    pub const HSENSOR_READ_USER_REG_COMMAND: u8 = 0xE7;

    // Processing constants
    /// temperature coefficient
    pub const HSENSOR_TEMPERATURE_COEFFICIENT: f32 = -0.15;
    /// constant a
    pub const HSENSOR_CONSTANT_A: f32 = 8.1332;
    /// constant b
    pub const HSENSOR_CONSTANT_B: f32 = 1762.39;
    /// constant c
    pub const HSENSOR_CONSTANT_C: f32 = 235.66;

    // Coefficients for temperature computation
    /// temperature coeff mul
    pub const TEMPERATURE_COEFF_MUL: f32 = 175.72;
    /// temperature coeff add
    pub const TEMPERATURE_COEFF_ADD: f32 = -46.85;

    // Coefficients for relative humidity computation
    /// humidity coeff mul
    pub const HUMIDITY_COEFF_MUL: f32 = 125.0;
    /// humidity coeff add
    pub const HUMIDITY_COEFF_ADD: f32 = -6.0;

    // Conversion timings
    /// conversion time 12b
    pub const HSENSOR_CONVERSION_TIME_12B: u8 = 16;
    /// conversion time 10b
    pub const HSENSOR_CONVERSION_TIME_10B: u8 = 5;
    /// conversion time 8b
    pub const HSENSOR_CONVERSION_TIME_8B: u8 = 3;
    /// conversion time 11b
    pub const HSENSOR_CONVERSION_TIME_11B: u8 = 9;

    // HSENSOR User Register masks and bit position
    /// user reg resolution mask
    pub const HSENSOR_USER_REG_RESOLUTION_MASK: u8 = 0x81;
    /// user reg end of battery mask
    pub const HSENSOR_USER_REG_END_OF_BATTERY_MASK: u8 = 0x40;
    /// user reg enable onchip heater mask
    pub const HSENSOR_USER_REG_ENABLE_ONCHIP_HEATER_MASK: u8 = 0x4;
    /// user reg disable otp reload mask
    pub const HSENSOR_USER_REG_DISABLE_OTP_RELOAD_MASK: u8 = 0x2;

    /// Humidity I2C address for the sensor.
    pub const RH_ADDRESS: u8 = 0x40;

    /// value for each count coming from the humidity
    /// register
    pub const MS8607_RH_LSB: f32 = 0.001_907_348_6;

    // PSENSOR commands
    /// 16-bit registers through 0xAE
    pub const PROM_ADDRESS_READ_ADDRESS_0: u8 = 0xA0;

    // Pressure & Temperature commands
    /// Pressure and temperature sensor reset
    pub const P_T_RESET: u8 = 0x1E;
    /// Sampling rate 256
    pub const CONVERT_D1_OSR_256: u8 = 0x40;
    /// Sampling rate 512
    pub const CONVERT_D1_OSR_512: u8 = 0x42;
    /// Sampling rate 1024
    pub const CONVERT_D1_OSR_1024: u8 = 0x44;
    /// Sampling rate 2048
    pub const CONVERT_D1_OSR_2048: u8 = 0x46;
    /// Sampling rate 4096
    pub const CONVERT_D1_OSR_4096: u8 = 0x48;
    /// Sampling rate 8192
    pub const CONVERT_D1_OSR_8192: u8 = 0x4A;
    /// Sampling rate 256
    pub const CONVERT_D2_OSR_256: u8 = 0x50;
    /// Sampling rate 512
    pub const CONVERT_D2_OSR_512: u8 = 0x52;
    /// Sampling rate 1024
    pub const CONVERT_D2_OSR_1024: u8 = 0x54;
    /// Sampling rate 2048
    pub const CONVERT_D2_OSR_2048: u8 = 0x56;
    /// Sampling rate 4096
    pub const CONVERT_D2_OSR_4096: u8 = 0x58;
    /// Sampling rate 8192
    pub const CONVERT_D2_OSR_8192: u8 = 0x5A;
    /// Command to read from ADC
    pub const ADC_READ: u8 = 0x00;
    // PROM READ P&T is from 0xA0 to 0xAE. Not sure whether or not to add

    // Commands for relative humidity
    /// Humidity sensor reset
    pub const HUMIDITY_RESET: u8 = 0xFE;
    /// Humidity sensor write register
    pub const HUMIDITY_WRITE_REGISTER: u8 = 0xE6;
    /// Humidity sensor read register
    pub const HUMIDITY_READ_REGISTER: u8 = 0xE7;
    /// Humidity sensor measure relative humidity hold master
    pub const HUM_MEASURE_RH_HOLD: u8 = 0xE5;
    /// Humidity sensor measure relative humidity no hold master
    pub const HUM_MEASURE_RH_NO_HOLD: u8 = 0xF5;
    // PROM READ RH is from 0xA0 to 0xAE. Not sure whether or not to add

    /// Command to reset pressure sensor
    pub const PSENSOR_RESET_COMMAND: u8 = 0x1E;
    /// Command to start pressure ADC measurement
    pub const PSENSOR_START_PRESSURE_ADC_CONVERSION: u8 = 0x40;
    /// Command to start temperature ADC measurement
    pub const PSENSOR_START_TEMPERATURE_ADC_CONVERSION: u8 = 0x50;
    /// Temp and pressure ADC read command
    pub const PSENSOR_READ_ADC: u8 = 0x00;
}

/// Pressure sensor resolution options
#[derive(Clone, Copy)]
pub enum Ms8607PressureResolution {
    /// 0
    Osr256,
    /// 1
    Osr512,
    /// 2
    Osr1024,
    /// 3
    Osr2048,
    /// 4
    Osr4096,
    /// 5
    Osr8192,
}

/// Options for setHumidityResolution
pub enum Ms8607HumidityResolution {
    Osr12b = 0x00,
    Osr11b = 0x81,
    Osr10b = 0x80,
    Osr8b = 0x01,
}

/// Options for I2C clock stretch for humidity readings
pub enum Ms8607HumidityClockStretch {
    Ms8607I2cHold = 0xE5,
    Ms8607I2cNoHold = 0xF5,
}
