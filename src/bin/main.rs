#![no_std]
#![no_main]
#![allow(clippy::style)]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_futures::select::Either;
use embassy_futures::select::select;
use embassy_futures::yield_now;
use embedded_graphics::primitives::PrimitiveStyleBuilder;
use minicontrol::press::HoldEvent;
use minicontrol::press::LongShortPress;
use minicontrol::press::PressEvent;
use minicontrol::twist::QwiicTwist;

use defmt::info;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;

use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_hal_bus::spi::NoDelay;
use esp_hal::clock::CpuClock;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use panic_rtt_target as _;

use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::I2c;
use esp_hal::spi::master::{Config, Spi};

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;

use embedded_graphics::{
    framebuffer::{Framebuffer, buffer_size},
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::{Rgb565, raw::LittleEndian},
    prelude::*,
    primitives::Rectangle,
    text::Text,
};

extern crate alloc;

use ssd1351::builder::Builder;
use ssd1351::mode::GraphicsMode;
use ssd1351::prelude::SPIInterface;
use static_cell::StaticCell;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

type I2cBus = Mutex<NoopRawMutex, I2c<'static, esp_hal::Async>>;
type Display = Mutex<
    NoopRawMutex,
    GraphicsMode<
        SPIInterface<
            ExclusiveDevice<Spi<'static, esp_hal::Blocking>, Output<'static>, NoDelay>,
            Output<'static>,
        >,
    >,
>;
type TwistCountChan = Channel<NoopRawMutex, i16, 1>;
type PressEventChan = Channel<NoopRawMutex, PressEvent, 1>;
type DisplayFrameBuffer = Mutex<
    NoopRawMutex,
    Framebuffer<Rgb565, RawU16, LittleEndian, 128, 128, { buffer_size::<Rgb565>(128, 128) }>,
>;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // generator version: 1.1.0

    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    info!("Embassy initialized!");

    // WiFi
    let wifi = esp_embassy_wifihelper::WifiStack::new(
        spawner,
        peripherals.WIFI,
        SSID.try_into().unwrap(),
        PASSWORD.try_into().unwrap(),
    );

    info!("Wifi initialized, waiting for connection...");
    let config = wifi.wait_for_connected().await.unwrap();
    info!("Wifi connected with IP: {}", config.address);

    // SPI Display Setup
    let spi_bus = Spi::new(
        peripherals.SPI2,
        Config::default().with_frequency(Rate::from_mhz(60)),
    )
    .expect("SPI should be initialized")
    .with_sck(peripherals.GPIO6)
    .with_mosi(peripherals.GPIO7);

    let cs = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let dc = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let mut rst = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());

    let spi_device = ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();
    let di = SPIInterface::new(spi_device, dc);
    let mut display: GraphicsMode<_> = Builder::new()
        .with_rotation(ssd1351::properties::DisplayRotation::Rotate0)
        .connect_interface(di)
        .into();

    display.reset(&mut rst, &mut Delay).unwrap();
    display.init().unwrap();

    // I2C Rotary Encoders Setup
    let config = esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, config)
        .unwrap()
        .with_sda(peripherals.GPIO8)
        .with_scl(peripherals.GPIO9)
        .into_async();

    static I2C_BUS: StaticCell<I2cBus> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    // Inter-task Communication
    static DISPLAY: StaticCell<Display> = StaticCell::new();
    static FRAMEBUFFER: StaticCell<DisplayFrameBuffer> = StaticCell::new();
    static LEFT_TWIST_COUNT_CHAN: StaticCell<TwistCountChan> = StaticCell::new();
    static RIGHT_TWIST_COUNT_CHAN: StaticCell<TwistCountChan> = StaticCell::new();
    static GREEN_BTN_CHAN: StaticCell<PressEventChan> = StaticCell::new();
    static FRAMEBUFFER_SIGNAL: StaticCell<Signal<NoopRawMutex, ()>> = StaticCell::new();

    let display = DISPLAY.init(Mutex::new(display));
    let framebuffer = FRAMEBUFFER.init(Mutex::new(Framebuffer::new()));
    let left_twist_count_chan = LEFT_TWIST_COUNT_CHAN.init(Channel::new());
    let right_twist_count_chan = RIGHT_TWIST_COUNT_CHAN.init(Channel::new());
    let green_btn_chan = GREEN_BTN_CHAN.init(Channel::new());
    let framebuffer_signal = FRAMEBUFFER_SIGNAL.init(Signal::new());

    // Spawn Tasks

    spawner
        .spawn(display_counts_task(
            left_twist_count_chan,
            right_twist_count_chan,
            framebuffer_signal,
            framebuffer,
        ))
        .unwrap();

    // spawner
    //     .spawn(animation_test_task(framebuffer_signal, framebuffer))
    //     .unwrap();

    spawner
        .spawn(green_button_task(
            Input::new(
                peripherals.GPIO21,
                InputConfig::default().with_pull(Pull::Up),
            ),
            green_btn_chan,
        ))
        .unwrap();

    spawner
        .spawn(count_twist_task(
            i2c_bus,
            0x3F,
            left_twist_count_chan,
            Duration::from_millis(100),
        ))
        .unwrap();

    spawner
        .spawn(count_twist_task(
            i2c_bus,
            0x45,
            right_twist_count_chan,
            Duration::from_millis(100),
        ))
        .unwrap();

    spawner.spawn(mode_switch_task(green_btn_chan)).unwrap();

    spawner
        .spawn(display_update_task(
            display,
            framebuffer,
            framebuffer_signal,
        ))
        .unwrap();
}

#[embassy_executor::task]
async fn mode_switch_task(mode_btn_chan: &'static PressEventChan) {
    // TODO: actually switch modes
    loop {
        match mode_btn_chan.receive().await {
            PressEvent::Held(HoldEvent::Short) => {
                info!("short held")
            }
            PressEvent::Held(HoldEvent::Long) => {
                info!("long held")
            }
            PressEvent::Released(hold, dur) => {
                info!("Released after {}ms held: {:?}", dur.as_millis(), hold)
            }
        }
    }
}

#[embassy_executor::task]
async fn display_update_task(
    display: &'static Display,
    framebuffer: &'static DisplayFrameBuffer,
    framebuffer_signal: &'static Signal<NoopRawMutex, ()>,
) {
    loop {
        framebuffer_signal.wait().await;
        let framebuffer = framebuffer.lock().await;
        let mut display = display.lock().await;
        embedded_graphics::image::Image::new(&framebuffer.as_image(), Point::new(0, 0))
            .draw(&mut *display)
            .unwrap();
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn count_twist_task(
    i2c_bus: &'static I2cBus,
    address: u8,
    count_chan: &'static TwistCountChan,
    freq: Duration,
) {
    let mut twist = QwiicTwist::new(I2cDevice::new(i2c_bus), address);
    let mut last_count = 0i16;
    let mut buf = [0u8; 2];

    loop {
        twist.get_count(&mut buf).await.unwrap();
        let count = i16::from_le_bytes(buf);
        if count != last_count {
            count_chan.send(count).await;
            last_count = count;
        }

        Timer::after(freq).await;
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn green_button_task(input: Input<'static>, green_press_chan: &'static PressEventChan) {
    let mut btn = LongShortPress::new(
        input,
        green_press_chan.sender(),
        Duration::from_secs(1),
        Duration::from_secs(5),
    );

    btn.listen().await;
}

#[embassy_executor::task]
async fn display_counts_task(
    left_twist_count_chan: &'static TwistCountChan,
    right_twist_count_chan: &'static TwistCountChan,
    framebuffer_signal: &'static Signal<NoopRawMutex, ()>,
    framebuffer: &'static DisplayFrameBuffer,
) {
    let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let rect_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::BLUE)
        .fill_color(Rgb565::WHITE)
        .build();

    let mut left_count: i16 = 0;
    let mut right_count: i16 = 0;
    let mut toggle = false;

    loop {
        match select(
            left_twist_count_chan.receive(),
            right_twist_count_chan.receive(),
        )
        .await
        {
            Either::First(count) => left_count = count,
            Either::Second(count) => right_count = count,
        }
        toggle = !toggle;

        {
            let mut framebuffer = framebuffer.lock().await;
            framebuffer.clear(RgbColor::BLACK);
            Text::new(
                &alloc::format!("Left: {} Right: {}", left_count, right_count),
                Point::new(0, 30),
                text_style,
            )
            .draw(&mut *framebuffer)
            .unwrap();

            if toggle {
                Rectangle::new(
                    Point { x: 100, y: 100 },
                    Size {
                        width: 28,
                        height: 28,
                    },
                )
                .into_styled(rect_style)
                .draw(&mut *framebuffer)
                .unwrap();
            }
        }
        framebuffer_signal.signal(());
    }
}

#[embassy_executor::task]
async fn animation_test_task(
    framebuffer_signal: &'static Signal<NoopRawMutex, ()>,
    framebuffer: &'static DisplayFrameBuffer,
) {
    let rect_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::BLUE)
        .fill_color(Rgb565::GREEN)
        .build();

    let mut x = 0;
    let mut y = 0;
    let mut dx = 1;
    let mut dy = 1;

    loop {
        {
            let mut framebuffer = framebuffer.lock().await;
            framebuffer.clear(RgbColor::BLACK);

            Rectangle::new(
                Point { x: x, y: y },
                Size {
                    width: 28,
                    height: 28,
                },
            )
            .into_styled(rect_style)
            .draw(&mut *framebuffer)
            .unwrap();

            x += dx;
            y += dy;

            if x >= 100 {
                dx = -1
            } else if x <= 0 {
                dx = 1
            }
            if y >= 100 {
                dy = -1
            } else if y <= 0 {
                dy = 1
            }
        }

        framebuffer_signal.signal(());
        yield_now().await;
    }
}
