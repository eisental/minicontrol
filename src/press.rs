use embassy_sync::{blocking_mutex::raw::NoopRawMutex, channel::Sender};
use embassy_time::{Duration, Instant, with_deadline};
use esp_hal::gpio::Input;

use crate::debounce::Debouncer;

#[derive(defmt::Format)]
pub enum HoldEvent {
    Short,
    Long,
}

pub enum PressEvent {
    Held(HoldEvent),
    Released(Option<HoldEvent>, Duration),
}

pub struct LongShortPress<'a> {
    debouncer: Debouncer<'a>,
    sender: Sender<'a, NoopRawMutex, PressEvent, 1>,
    short_press: Duration,
    long_press: Duration,
}

impl<'a> LongShortPress<'a> {
    pub fn new(
        input: Input<'a>,
        sender: Sender<'a, NoopRawMutex, PressEvent, 1>,
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

    fn elapsed_to_hold_event(&self, elapsed: Duration) -> Option<HoldEvent> {
        if elapsed > self.long_press {
            Some(HoldEvent::Long)
        } else if elapsed > self.short_press {
            Some(HoldEvent::Short)
        } else {
            None
        }
    }

    pub async fn listen(&mut self) {
        loop {
            self.debouncer.debounce().await;
            let start = Instant::now();

            match with_deadline(start + self.short_press, self.debouncer.debounce()).await {
                Ok(_) => {
                    self.sender
                        .send(PressEvent::Released(
                            self.elapsed_to_hold_event(start.elapsed()),
                            start.elapsed(),
                        ))
                        .await;

                    continue;
                }
                Err(_) => {
                    self.sender.send(PressEvent::Held(HoldEvent::Short)).await;
                }
            }

            match with_deadline(start + self.long_press, self.debouncer.debounce()).await {
                Ok(_) => {
                    self.sender
                        .send(PressEvent::Released(
                            self.elapsed_to_hold_event(start.elapsed()),
                            start.elapsed(),
                        ))
                        .await;

                    continue;
                }
                Err(_) => {
                    self.sender.send(PressEvent::Held(HoldEvent::Long)).await;
                }
            }

            self.debouncer.debounce().await;
            self.sender
                .send(PressEvent::Released(
                    self.elapsed_to_hold_event(start.elapsed()),
                    start.elapsed(),
                ))
                .await;
        }
    }
}
