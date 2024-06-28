#![no_std]
#![no_main]

use bsp::entry;
use core::fmt::Write as _;
use core::panic::PanicInfo;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use fugit::{self, RateExtU32};
use num_traits::ToPrimitive;
use rp_pico::hal::gpio::bank0::{Gpio4, Gpio5};
use rp_pico::hal::gpio::{FunctionI2c, PullUp};
use rp_pico::pac::I2C0;

use rp_pico::hal;
use rp_pico::{self as bsp, hal::gpio::Pin, hal::I2C};

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    pac::interrupt,
    sio::Sio,
    watchdog::Watchdog,
};

mod usb;
use usb::Usb;

static mut USB_INSTANCE: Option<Usb> = None;

#[interrupt]
unsafe fn USBCTRL_IRQ() {
    match USB_INSTANCE.as_mut() {
        Some(usb_instance) => {
            if usb_instance
                .usb_device
                .poll(&mut [&mut usb_instance.serial])
            {}
        }
        _ => (),
    }
}

#[panic_handler]
unsafe fn panic_handler(info: &PanicInfo) -> ! {
    match USB_INSTANCE.as_mut() {
        Some(usb_instance) => {
            write!(usb_instance, "\n{}\n", info).unwrap();
        }
        _ => (),
    }

    loop {}
}

fn read_moisture(
    i2c: &mut I2C<
        I2C0,
        (
            Pin<Gpio4, FunctionI2c, PullUp>,
            Pin<Gpio5, FunctionI2c, PullUp>,
        ),
    >,
    address: u8,
    delay: &mut cortex_m::delay::Delay,
) -> u32 {
    let module_base_address = 0x0Fu8;
    let moisture_register_address = 0x10u8;
    let writebuf = [module_base_address, moisture_register_address];

    embedded_hal::i2c::I2c::write(i2c, address, &writebuf).unwrap();

    delay.delay_ms(5);

    let mut readbuf: [u8; 2] = [0; 2];
    embedded_hal::i2c::I2c::read(i2c, address, &mut readbuf).unwrap();
    (readbuf[0] as u32) * readbuf[1] as u32
}

fn read_temperature(
    i2c: &mut I2C<
        I2C0,
        (
            Pin<Gpio4, FunctionI2c, PullUp>,
            Pin<Gpio5, FunctionI2c, PullUp>,
        ),
    >,
    address: u8,
    delay: &mut cortex_m::delay::Delay,
) -> f32 {
    let status_base_address = 0x00u8;
    let temperature_register_address = 0x04u8;
    let writebuf = [status_base_address, temperature_register_address];

    embedded_hal::i2c::I2c::write(i2c, address, &writebuf).unwrap();

    delay.delay_ms(5);

    let mut readbuf: [u8; 4] = [0; 4];
    embedded_hal::i2c::I2c::read(i2c, address, &mut readbuf).unwrap();
    match (((readbuf[0] as u32) & 0x3F) << 24
        | (readbuf[1] as u32) << 16
        | (readbuf[2] as u32) << 8
        | readbuf[3] as u32)
        .to_f32()
    {
        Some(result) => result * 0.00001525878,
        None => 0.0,
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let usb = unsafe {
        USB_INSTANCE = Some(Usb::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        ));
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
        USB_INSTANCE.as_mut().unwrap()
    };

    let mut led_pin = pins.led.into_push_pull_output();

    let i2c_serial_freq = 100_000u32;
    let mut i2c = I2C::i2c0(
        pac.I2C0,
        pins.gpio4.reconfigure(),
        pins.gpio5.reconfigure(),
        i2c_serial_freq.Hz(),
        &mut pac.RESETS,
        clocks.system_clock.freq(),
    );

    delay.delay_ms(5000);

    let mut device_address: Option<u8> = None;
    for i in 0..=127u8 {
        let mut readbuf: [u8; 1] = [0; 1];
        let result = embedded_hal::i2c::I2c::read(&mut i2c, i, &mut readbuf);
        if result.is_ok() {
            device_address = Some(i);
            write!(usb, "Device found at address {:?}", i).unwrap();
        }
    }

    loop {
        if let Some(address) = device_address {
            let moisture = read_moisture(&mut i2c, address, &mut delay);
            let temperature = read_temperature(&mut i2c, address, &mut delay);
            write!(
                usb,
                "Moisture: {}, Temperature: {}°C.",
                moisture, temperature
            )
            .unwrap();
        }

        led_pin.set_high().unwrap();
        delay.delay_ms(500);

        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}
