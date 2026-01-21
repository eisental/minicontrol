pub struct QwiicTwist<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C: embedded_hal_async::i2c::I2c> QwiicTwist<I2C> {
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    pub async fn get_count(&mut self) -> i16 {
        let mut buffer = [0u8; 2];
        // Register 0x05 is the Encoder Count (2 bytes, little endian)
        let _ = self
            .i2c
            .write_read(self.address, &[0x05], &mut buffer)
            .await;
        i16::from_le_bytes(buffer)
    }

    pub async fn set_color(&mut self, r: u8, g: u8, b: u8) {
        // Registers 0x0D, 0x0E, 0x0F are Red, Green, Blue
        let _ = self.i2c.write(self.address, &[0x0D, r, g, b]).await;
    }
}
