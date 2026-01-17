#![no_std]

use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Sender};
use embassy_time::{Duration, Instant, Timer, with_deadline};
use esp_hal::gpio::{Input, Level};

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

pub struct Debouncer<'a> {
    input: Input<'a>,
    debounce: Duration,
}

impl<'a> Debouncer<'a> {
    pub fn new(input: Input<'a>, debounce: Duration) -> Self {
        Self { input, debounce }
    }

    pub async fn debounce(&mut self) -> Level {
        loop {
            let l1 = self.input.level();
            self.input.wait_for_any_edge().await;
            Timer::after(self.debounce).await;
            let l2 = self.input.level();
            if l1 != l2 {
                break l2;
            }
        }
    }
}

#[derive(Clone, Copy)]
pub enum LongShortPressEvent {
    ShortHeld,
    LongHeld,
    Released(Duration),
}

pub struct LongShortPress<'a> {
    debouncer: Debouncer<'a>,
    sender: Sender<'a, NoopRawMutex, LongShortPressEvent, 1>,
    short_press: Duration,
    long_press: Duration,
}

impl<'a> LongShortPress<'a> {
    pub fn new(
        input: Input<'a>,
        sender: Sender<'a, NoopRawMutex, LongShortPressEvent, 1>,
        short_press: Duration,
        long_press: Duration,
    ) -> Self {
        let debouncer = Debouncer::new(input, Duration::from_millis(20));

        Self {
            debouncer,
            sender,
            short_press,
            long_press,
        }
    }

    pub async fn listen(&mut self) {
        loop {
            self.debouncer.debounce().await;
            let start = Instant::now();

            match with_deadline(start + self.short_press, self.debouncer.debounce()).await {
                Ok(_) => {
                    self.sender
                        .send(LongShortPressEvent::Released(start.elapsed()))
                        .await;

                    continue;
                }
                Err(_) => {
                    self.sender.send(LongShortPressEvent::ShortHeld).await;
                }
            }

            match with_deadline(start + self.long_press, self.debouncer.debounce()).await {
                Ok(_) => {
                    self.sender
                        .send(LongShortPressEvent::Released(start.elapsed()))
                        .await;

                    continue;
                }
                Err(_) => {
                    self.sender.send(LongShortPressEvent::LongHeld).await;
                }
            }

            self.debouncer.debounce().await;
            self.sender
                .send(LongShortPressEvent::Released(start.elapsed()))
                .await;
        }
    }
}
