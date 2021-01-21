# lis3mdl
This is a library for interacting over i2c with the LIS3MDL magnetometer. It is based on the [manufacterer's aruduino specific library](https://github.com/pololu/lis3mdl-arduino) and the [datasheet](https://www.pololu.com/file/0J1089/LIS3MDL.pdf). The methods that this library provide are abstracted away from a specific i2c implementation using traits from [embedded-hal](https://crates.io/crates/embedded-hal).
