pub mod registers;

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

const LIS3MDL_SA1_HIGH_ADDRESS: u8 = 0b0011110;
const LIS3MDL_SA1_LOW_ADDRESS: u8 = 0b0011100;

const LIS3MDL_WHO_ID: u8 = 0x3d;

pub enum OperatingMode {
    ContinuousConversion,
    SingleConversion,
    PowerDown,
}

impl OperatingMode {
    fn to_bitcode(self) -> u8 {
        match self {
            OperatingMode::ContinuousConversion => 0,
            OperatingMode::SingleConversion => 1,
            OperatingMode::PowerDown => 2,
        }
    }
}

pub enum FullScale {
    Four,
    Eight,
    Twelve,
    Sixteen,
}

impl FullScale {
    pub fn to_bitcode(self) -> u8 {
        match self {
            FullScale::Four => 0,
            FullScale::Eight => 1,
            FullScale::Twelve => 2,
            FullScale::Sixteen => 3,
        }
    }
}

pub enum AxisMode {
    LowPower,
    MediumPerformance,
    HighPerformance,
    UltraPerformance,
}

impl AxisMode {
    fn to_bitcode(self) -> u8 {
        match self {
            AxisMode::LowPower => 0,
            AxisMode::MediumPerformance => 1,
            AxisMode::HighPerformance => 2,
            AxisMode::UltraPerformance => 3,
        }
    }
}

pub enum OutputDataRate {
    Fast,
    MilliHz625,
    MilliHz1250,
    MilliHz2500,
    Hz5,
    Hz10,
    Hz20,
    Hz40,
    Hz80,
}

impl OutputDataRate {
    fn to_bitcode(self) -> u8 {
        match self {
            OutputDataRate::Fast => 1,
            OutputDataRate::MilliHz625 => 0,
            OutputDataRate::MilliHz1250 => 0b10,
            OutputDataRate::MilliHz2500 => 0b100,
            OutputDataRate::Hz5 => 0b110,
            OutputDataRate::Hz10 => 0b1000,
            OutputDataRate::Hz20 => 0b1010,
            OutputDataRate::Hz40 => 0b1100,
            OutputDataRate::Hz80 => 0b1110,
        }
    }
}

pub struct LIS3MDL<E, I: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>> {
    address: u8,
    i2c: I,
}

impl<E, I: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>> LIS3MDL<E, I> {
    pub fn new(mut i2c: I) -> Result<Option<Self>, E> {
        // Get the correct address for the LIS3MDL that is being used
        let address = if test_lism3mdl_addr(&mut i2c, LIS3MDL_SA1_HIGH_ADDRESS)? {
            LIS3MDL_SA1_HIGH_ADDRESS
        } else if test_lism3mdl_addr(&mut i2c, LIS3MDL_SA1_LOW_ADDRESS)? {
            LIS3MDL_SA1_LOW_ADDRESS
        } else {
            return Ok(None);
        };

        let this = Self { address, i2c };
        // TODO: unsure if I have to turn on incrementation like I did for the lsm6ds33

        Ok(Some(this))
    }

    /// Initialize the lis3mdl in high performance axis modes, continous conversion mode, 
    /// and 10 Hz output data rate.
    pub fn init_default(&mut self) -> Result<(), E> {
        self.set_xy_mode_and_data_rate(AxisMode::HighPerformance, OutputDataRate::Hz10)?;
        self.set_z_mode(AxisMode::HighPerformance)?;
        self.set_operating_mode(OperatingMode::ContinuousConversion)
    }

    /// Sets the operating mode for the whole system. This is entirely different than setting the xy mode or z mode.
    /// This overwrites the CTRL_REG3 register.
    pub fn set_operating_mode(&mut self, mode: OperatingMode) -> Result<(), E> {
        self.set_register(registers::CTRL_REG3, mode.to_bitcode())
    }

    /// Alias for `set_operating_mode(OperatingMode::PowerDown)`.
    pub fn power_down(&mut self) -> Result<(), E> {
        self.set_operating_mode(OperatingMode::PowerDown)
    }

    /// Sets the full scale (in ± gauss) of the magnetometer.
    /// Overwrites the CTRL_REG2 register.
    pub fn set_full_scale(&mut self, scale: FullScale) -> Result<(), E> {
        self.set_register(registers::CTRL_REG2, scale.to_bitcode())
    }

    /// Set the operative mode for the x and y axes as well as the output data rate of the sensor.
    /// This function is faster than setting both individually, but will fully overwrite the CTRL_REG1 register.
    pub fn set_xy_mode_and_data_rate(
        &mut self,
        mode: AxisMode,
        odr: OutputDataRate,
    ) -> Result<(), E> {
        self.set_register(
            registers::CTRL_REG1,
            (mode.to_bitcode() << 5) | (odr.to_bitcode() << 1),
        )
    }

    /// Sets the operative mode of the x and y axes while
    /// only overwriting the relevant bits of the CTRL_REG1 register.
    pub fn set_xy_mode(&mut self, mode: AxisMode) -> Result<(), E> {
        let cur = self.read_register(registers::CTRL_REG1)?;
        self.set_register(registers::CTRL_REG1, cur & (0b10011111 & (mode.to_bitcode() << 5)))
    }

    /// Sets the operative mode fo the z axis.
    /// Overwrites the CTRL_REG4 register.
    pub fn set_z_mode(&mut self, mode: AxisMode) -> Result<(), E> {
        self.set_register(registers::CTRL_REG4, mode.to_bitcode() << 2)
    }

    /// Sets the output data rate while
    /// only overwriting the relevant bits of the CTRL_REG1 register.
    pub fn set_data_rate(&mut self, odr: OutputDataRate) -> Result<(), E> {
        let cur = self.read_register(registers::CTRL_REG1)?;
        self.set_register(registers::CTRL_REG1, cur & (0b11100001 & (odr.to_bitcode() << 1)))
    }

    /// Set one of the LIS3MDL's register to a certain value
    pub fn set_register(&mut self, reg: u8, value: u8) -> Result<(), E> {
        self.i2c.write(self.address, &[reg, value])
    }

    /// Read one of the LIS3MDL's registers
    pub fn read_register(&mut self, reg: u8) -> Result<u8, E> {
        let mut resp = [0];
        self.i2c.write_read(self.address, &[reg], &mut resp)?;
        Ok(resp[0])
    }

    /// Reads the latest data, returning `Ok(None)` if any is not ready.
    /// A `None` return does not necessarily indicate that anything has failed,
    /// and this function can be called immediately afterwards.
    pub fn read(&mut self) -> Result<Option<(i16, i16, i16)>, E> {
        if self.read_register(registers::STATUS_REG)? & 0b1000 != 0b1000 {
            return Ok(None);
        }
        self.incremental_read_measurements(registers::OUT_X_L)
            .map(|o| Some(o))
    }

    /// This method of extracting measurements only works if the 2nd bit (0-indexed) of the CTRL_3C register is set to 1.
    fn incremental_read_measurements(&mut self, start_reg: u8) -> Result<(i16, i16, i16), E> {
        let mut values = [0; 6];
        self.i2c
            .write_read(self.address, &[start_reg], &mut values)?;

        Ok((
            (values[1] as i16) << 8 | values[0] as i16,
            (values[3] as i16) << 8 | values[2] as i16,
            (values[5] as i16) << 8 | values[4] as i16,
        ))
    }
}

fn test_lism3mdl_addr<I: WriteRead>(i2c: &mut I, address: u8) -> Result<bool, I::Error> {
    let mut resp = [LIS3MDL_WHO_ID + 1];
    i2c.write_read(address, &[registers::WHO_AM_I], &mut resp)?;
    Ok(resp[0] == LIS3MDL_WHO_ID)
}
