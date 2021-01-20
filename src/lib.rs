pub mod registers;

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

const LIS3MDL_SA1_HIGH_ADDRESS: u8 = 0b0011110;
const LIS3MDL_SA1_LOW_ADDRESS: u8 =  0b0011100;

const LIS3MDL_WHO_ID: u8 = 0x3d;

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

        let mut this = Self { address, i2c };

        Ok(Some(this))
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

    /// Reads the latest acceleration data, returning `Ok(None)` if any is not ready.
    /// A `None` return does not necessarily indicate that anything has failed,
    /// and this function can be called immediately afterwards.
    /// This method of extracting measurements only works if the 2nd bit (0-indexed) of the CTRL_3C register is set to 1
    /// (which automatically happens in `LSMG::new`).
    pub fn read_gyro(&mut self) -> Result<Option<(i16, i16, i16)>, E> {
        if self.read_register(registers::STATUS_REG)? & 0b10 != 0b10 {
            return Ok(None);
        }
        self.incremental_read_measurements(registers::OUT_X_L).map(|o| Some(o))
    }

    /// This method of extracting measurements only works if the 2nd bit (0-indexed) of the CTRL_3C register is set to 1.
    fn incremental_read_measurements(
        &mut self,
        start_reg: u8,
    ) -> Result<(i16, i16, i16), E> {
        let mut values = [0; 6];
        self.i2c.write_read(self.address, &[start_reg], &mut values)?;

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
