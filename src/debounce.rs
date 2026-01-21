use embassy_time::{Duration, Timer};
use esp_hal::gpio::{Input, Level};

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
