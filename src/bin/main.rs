#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use minicontrol::LongShortPress;
use minicontrol::LongShortPressEvent;
use minicontrol::QwiicTwist;

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

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
type EncoderCountChan = Channel<NoopRawMutex, (i16, i16), 1>;
type PressEventChan = Channel<NoopRawMutex, LongShortPressEvent, 1>;
type DisplayFrameBuffer = Mutex<
    NoopRawMutex,
    Framebuffer<Rgb565, RawU16, LittleEndian, 128, 128, { buffer_size::<Rgb565>(128, 128) }>,
>;

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

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    // SPI Display Setup
    let spi_bus = Spi::new(
        peripherals.SPI2,
        Config::default().with_frequency(Rate::from_mhz(4)),
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
    static ENCODER_COUNT_CHAN: StaticCell<EncoderCountChan> = StaticCell::new();
    static GREEN_BTN_CHAN: StaticCell<PressEventChan> = StaticCell::new();
    static FRAMEBUFFER_SIGNAL: StaticCell<Signal<NoopRawMutex, ()>> = StaticCell::new();

    let display = DISPLAY.init(Mutex::new(display));
    let framebuffer = FRAMEBUFFER.init(Mutex::new(Framebuffer::new()));
    let encoder_count_chan = ENCODER_COUNT_CHAN.init(Channel::new());
    let green_btn_chan = GREEN_BTN_CHAN.init(Channel::new());
    let framebuffer_signal = FRAMEBUFFER_SIGNAL.init(Signal::new());

    // Spawn Tasks
    spawner
        .spawn(display_counts_task(
            encoder_count_chan,
            framebuffer_signal,
            framebuffer,
        ))
        .unwrap();

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
        .spawn(encoder_task(i2c_bus, encoder_count_chan))
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
            LongShortPressEvent::ShortHeld => {
                info!("short held")
            }
            LongShortPressEvent::LongHeld => {
                info!("long held")
            }
            LongShortPressEvent::Released(dur) => {
                info!("Released after {}ms", dur.as_millis())
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
async fn encoder_task(i2c_bus: &'static I2cBus, encoder_count: &'static EncoderCountChan) {
    let mut twist_left = QwiicTwist::new(I2cDevice::new(i2c_bus), 0x3F);
    let mut twist_right = QwiicTwist::new(I2cDevice::new(i2c_bus), 0x45);

    loop {
        let left_count = twist_left.get_count().await;
        let right_count = twist_right.get_count().await;

        encoder_count.send((left_count, right_count)).await;

        Timer::after(Duration::from_millis(100)).await;
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
    encoder_count_chan: &'static EncoderCountChan,
    framebuffer_signal: &'static Signal<NoopRawMutex, ()>,
    framebuffer: &'static DisplayFrameBuffer,
) {
    let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);

    let mut last_encoder_left: i16 = 0;
    let mut last_encoder_right: i16 = 0;

    loop {
        let (encoder_left, encoder_right) = encoder_count_chan.receive().await;
        if encoder_left == last_encoder_left && encoder_right == last_encoder_right {
            continue;
        }
        (last_encoder_left, last_encoder_right) = (encoder_left, encoder_right);

        {
            let mut framebuffer = framebuffer.lock().await;
            framebuffer.clear(RgbColor::BLACK);
            Text::new(
                &alloc::format!("Left: {} Right: {}", last_encoder_left, last_encoder_right),
                Point::new(0, 30),
                style,
            )
            .draw(&mut *framebuffer)
            .unwrap();
        }
        framebuffer_signal.signal(());
    }
}
