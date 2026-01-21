use embedded_hal_async::i2c::I2c;

pub struct QwiicTwist<T: I2c> {
    i2c: T,
    address: u8,
}

impl<T: I2c> QwiicTwist<T> {
    pub fn new(i2c: T, address: u8) -> Self {
        Self { i2c, address }
    }

    pub async fn get_count(&mut self, buf: &mut [u8; 2]) -> Result<(), T::Error> {
        // Register 0x05 is the Encoder Count (2 bytes, little endian)
        self.i2c.write_read(self.address, &[0x05], buf).await
    }

    pub async fn set_color(&mut self, r: u8, g: u8, b: u8) -> Result<(), T::Error> {
        // Registers 0x0D, 0x0E, 0x0F are Red, Green, Blue
        self.i2c.write(self.address, &[0x0D, r, g, b]).await
    }
}
