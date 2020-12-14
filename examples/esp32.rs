#![no_std]
#![no_main]

use core::cmp;
use core::fmt::Write;
use core::convert::Infallible;
use xtensa_lx::timer::delay;
//use panic_halt as _;
use esp32_hal as hal;


use hal::{
    clock_control::{sleep, ClockControl, XTAL_FREQUENCY_AUTO, CPUSource},
    dport::Split,
    dprintln,
    gpio::{self, InputPin, OutputPin},
    prelude::*,
    serial::{self, Serial},
    spi::{self, SPI},
    target,
    timer::Timer,
};
use _embedded_hal_blocking_spi_Write as SpiWrite;
use embedded_graphics::{egtext, fonts, pixelcolor::Rgb888, prelude::*, text_style, primitives::{Triangle, Circle}, style::PrimitiveStyle};

/// The default clock source is the onboard crystal
/// In most cases 40mhz (but can be as low as 2mhz depending on the board)
const CORE_HZ: u32 = 10_000_000;

const WDT_WKEY_VALUE: u32 = 0x50D83AA1;

#[entry]
fn main() -> ! {
    let dp = target::Peripherals::take().expect("Failed to obtain Peripherals");

    let mut rtccntl = dp.RTCCNTL;
    let mut timg0 = dp.TIMG0;
    let mut timg1 = dp.TIMG1;
    let pins = dp.GPIO.split();
    let (mut dport, dport_clock_control) = dp.DPORT.split();

    // (https://github.com/espressif/openocd-esp32/blob/97ba3a6bb9eaa898d91df923bbedddfeaaaf28c9/src/target/esp32.c#L431)
    // openocd disables the wdt's on halt
    // we will do it manually on startup
    disable_timg_wdts(&mut timg0, &mut timg1);
    disable_rtc_wdt(&mut rtccntl);
    
    let mut clock_control = hal::clock_control::ClockControl::new(rtccntl, dp.APB_CTRL, dport_clock_control, XTAL_FREQUENCY_AUTO).unwrap();

/*
    clock_control
        .set_cpu_frequencies(
            CPUSource::Xtal,
            10.MHz(),
            CPUSource::Xtal,
            240.MHz(),
            CPUSource::PLL,
            80.MHz(),
        )
        .unwrap();
*/

    let (clock_control_config, mut watchdog) = clock_control.freeze().unwrap(); 

    let mut serial: hal::serial::Serial<_, _, _> = hal::serial::Serial::new(dp.UART0, hal::serial::Pins {
        tx: pins.gpio1,
        rx: pins.gpio3,
        cts: None,
        rts: None,
    }, hal::serial::config::Config {
        baudrate: 115200.into(),
        data_bits: hal::serial::config::DataBits::DataBits8,
        parity: hal::serial::config::Parity::ParityNone,
        stop_bits: hal::serial::config::StopBits::STOP1,
    }, clock_control_config).unwrap();
    
    //let (tx, rx) = serial.split();
    
    let mut spi = SPI::<target::SPI2, _, _, gpio::Gpio0<gpio::Unknown>, _>::new(
        dp.SPI2,
        spi::Pins {
            sclk: pins.gpio18,
            sdo: pins.gpio23,
            sdi: None,
            cs: Some(pins.gpio5),
        },
        spi::config::Config {
            baudrate: 4.MHz().into(),
            bit_order: spi::config::BitOrder::MSBFirst,
            data_mode: spi::config::MODE_0,
        },
        clock_control_config,
    )
    .unwrap();



    let mut led = pins.gpio19.into_push_pull_output();

    //let mut cs = pins.gpio5.into_push_pull_output();
    let mut dc = pins.gpio17.into_push_pull_output();
    let mut rst = pins.gpio16.into_push_pull_output();
    let mut busy = pins.gpio4.into_floating_input();
    
    let mut delay = hal::delay::Delay::new();
    
    let e_ink_interface = ssd1675::Interface::new(spi, /*cs*/led, busy, dc, rst);
    let display_config = ssd1675::Builder::new().dimensions(ssd1675::Dimensions { rows: 250, cols: 128 }).rotation(ssd1675::Rotation::Rotate270).lut(&LUT_DATA_FULL).build().unwrap();
    let display = ssd1675::Display::new(e_ink_interface, display_config);
    
    let mut black_buffer = [0u8; (128 * 250) / 8];
    let mut red_buffer = [0u8; (128 * 250) / 8];
    
    let mut display = ssd1675::GraphicDisplay::new(display, &mut black_buffer, &mut red_buffer);
    
    

    display.reset(&mut delay).unwrap();
    display.clear(ssd1675::Color::White);
    
    let max_x = 136;
    let max_y = 96;
    let mut dir_x = 10i32;
    let mut dir_y = 10i32;
    let mut x = 80;
    let mut y = 40;
    loop {
    x += dir_x;
    if (x >= max_x) || (x <= 0) { dir_x *= -1; }
    y += dir_y;
    if (y >= max_y) || (y <= 0) { dir_y *= -1; }
    
    
    
    display.clear(ssd1675::Color::White);
    egtext!(
        text = "MEMES",
        //top_left = (0, 0),
        top_left = (cmp::min(max_x, cmp::max(0, x)), cmp::min(max_y, cmp::max(0, y))),
        //top_left = (80, 40),
        style = text_style!(
            font = fonts::Font24x32,
            background_color = ssd1675::Color::White,
            text_color = ssd1675::Color::Black,
        )
    ).draw(&mut display).unwrap();
    //Triangle::new(Point::new(40, 60), Point::new(30, 30), Point::new(50, 30)).into_styled(PrimitiveStyle::with_fill(ssd1675::Color::Black)).draw(&mut display).unwrap();
    //Circle::new(Point::new(40, 80), 5).into_styled(PrimitiveStyle::with_fill(ssd1675::Color::Black)).draw(&mut display).unwrap();
    
    display.update(&mut delay).unwrap();
    //display.deep_sleep().unwrap();
    //writeln!(serial, "white\r").unwrap();
    hal::clock_control::sleep(500.ms());
    }
/*
loop {    
    display.reset(&mut delay).unwrap();
    display.clear(ssd1675::Color::White);
    display.update(&mut delay).unwrap();
    display.deep_sleep().unwrap();
    writeln!(serial, "white\r").unwrap();
    hal::clock_control::sleep(5000.ms());
    
    
    
    display.reset(&mut delay).unwrap();
    display.clear(ssd1675::Color::Black);
    display.update(&mut delay).unwrap();
    display.deep_sleep().unwrap();
    writeln!(serial, "black\r").unwrap();
    hal::clock_control::sleep(5000.ms());
    }
*/    

    /*
    //cs.set_high().unwrap();
    dc.set_high().unwrap();
    rst.set_high().unwrap();
    
    writeln!(serial, "resetting\r").unwrap();
    hal::clock_control::sleep(20.ms());
    rst.set_low().unwrap();
    hal::clock_control::sleep(20.ms());
    rst.set_high().unwrap();
    hal::clock_control::sleep(200.ms());
    
    write_command(0x74, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x54, &mut spi).unwrap();
    write_command(0x7E, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x3B, &mut spi).unwrap();
    write_command(0x01, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0xF9, &mut spi).unwrap();
    write_data(0x00, &mut spi).unwrap();
    write_data(0x00, &mut spi).unwrap();
    write_command(0x11, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x01, &mut spi).unwrap();
    write_command(0x44, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x00, &mut spi).unwrap();
    write_data(0x0F, &mut spi).unwrap();
    write_command(0x45, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0xF9, &mut spi).unwrap();
    write_data(0x00, &mut spi).unwrap();
    write_data(0x00, &mut spi).unwrap();
    write_data(0x00, &mut spi).unwrap();
    write_command(0x3C, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x03, &mut spi).unwrap();
    write_command(0x2C, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x50, &mut spi).unwrap();
    write_command(0x03, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x15, &mut spi).unwrap();
    write_command(0x04, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x41, &mut spi).unwrap();
    write_data(0xA8, &mut spi).unwrap();
    write_data(0x32, &mut spi).unwrap();
    write_command(0x3A, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x2C, &mut spi).unwrap();
    write_command(0x3B, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x0B, &mut spi).unwrap();
    write_command(0x4E, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x00, &mut spi).unwrap();
    write_command(0x4F, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0xF9, &mut spi).unwrap();
    write_data(0x00, &mut spi).unwrap();


/*
the original code sets the parameters twice, no idea why

    write_command(0x11, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x01, &mut spi, &mut busy, &mut dc).unwrap();
    write_command(0x44, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x00, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x0F, &mut spi, &mut busy, &mut dc).unwrap();
    write_command(0x45, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x01, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x01, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x00, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0x00, &mut spi, &mut busy, &mut dc).unwrap();
*/
    write_command(0x32, &mut spi, &mut busy, &mut dc).unwrap();
    spi.write(&LUT_DATA_FULL).unwrap();
    
    write_command(0x22, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0xc0, &mut spi).unwrap();
    write_command(0x20, &mut spi, &mut busy, &mut dc).unwrap();
    while busy.is_high().unwrap() {}
    
    write_command(0x24, &mut spi, &mut busy, &mut dc).unwrap();
    for row in 0..250 {
        for col in 0..16 {
            let s = row & 1;
            write_data(0x55 << s, &mut spi).unwrap();
        }
    }
    write_command(0x26, &mut spi, &mut busy, &mut dc).unwrap();
    for row in 0..250 {
        for col in 0..16 {
            let s = row & 1;
            write_data(0x55 << s, &mut spi).unwrap();
        }
    }

    write_command(0x22, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0xc7, &mut spi).unwrap();
    write_command(0x20, &mut spi, &mut busy, &mut dc).unwrap();
    while busy.is_high().unwrap() {}

    write_command(0x22, &mut spi, &mut busy, &mut dc).unwrap();
    write_data(0xc3, &mut spi).unwrap();
    write_command(0x20, &mut spi, &mut busy, &mut dc).unwrap();
    while busy.is_high().unwrap() {}
    */

    loop {
    /*
        led.set_high().unwrap();
        writeln!(serial, "memes H\r").unwrap();
        delay(CORE_HZ);
        led.set_low().unwrap();
        writeln!(serial, "memes L\r").unwrap();
        delay(CORE_HZ);
        */
    }
}

const LUT_DATA_FULL: [u8; 100] =  [0xA0,  0x90, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x50, 0x90, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xA0, 0x90, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x50, 0x90, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  0x0F, 0x0F, 0x00, 0x00, 0x00,
  0x0F, 0x0F, 0x00, 0x00, 0x03,
  0x0F, 0x0F, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00,
  ]
;


fn write_command(cmd: u8, spi: &mut impl SpiWrite<u8, Error=spi::Error>, busy: &mut impl embedded_hal::digital::v2::InputPin<Error=Infallible>, dc: &mut impl embedded_hal::digital::v2::OutputPin<Error=Infallible>) -> Result<(), spi::Error> {
    while busy.is_high().unwrap() {}
    dc.set_low().unwrap();
    write_data(cmd, spi)?;
    dc.set_high().unwrap();
    Ok(())
}

fn write_data(cmd: u8, spi: &mut impl SpiWrite<u8, Error=spi::Error>) -> Result<(), spi::Error> {
    spi.write(&[cmd])
}

fn disable_rtc_wdt(rtccntl: &mut target::RTCCNTL) {
    /* Disables the RTCWDT */
    rtccntl
        .wdtwprotect
        .write(|w| unsafe { w.bits(WDT_WKEY_VALUE) });
    rtccntl.wdtconfig0.modify(|_, w| unsafe {
        w.wdt_stg0()
            .bits(0x0)
            .wdt_stg1()
            .bits(0x0)
            .wdt_stg2()
            .bits(0x0)
            .wdt_stg3()
            .bits(0x0)
            .wdt_flashboot_mod_en()
            .clear_bit()
            .wdt_en()
            .clear_bit()
    });
    rtccntl.wdtwprotect.write(|w| unsafe { w.bits(0x0) });
}

fn disable_timg_wdts(timg0: &mut target::TIMG0, timg1: &mut target::TIMG1) {
    timg0
        .wdtwprotect
        .write(|w| unsafe { w.bits(WDT_WKEY_VALUE) });
    timg1
        .wdtwprotect
        .write(|w| unsafe { w.bits(WDT_WKEY_VALUE) });

    timg0.wdtconfig0.write(|w| unsafe { w.bits(0x0) });
    timg1.wdtconfig0.write(|w| unsafe { w.bits(0x0) });
}

use core::panic::PanicInfo;
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    dprintln!("\n\n*** {:?}", info);
    loop {}
}
