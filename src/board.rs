use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use fugit::{self, RateExtU32};
use num_traits::ToPrimitive;
use rp_pico::hal::gpio::bank0::{Gpio2, Gpio25, Gpio4, Gpio5};
use rp_pico::hal::gpio::{FunctionI2c, FunctionSio, Pin, PullDown, PullUp, SioOutput};
use rp_pico::hal::{usb::UsbBus, I2C};
use rp_pico::pac::I2C0;
use rp_pico::{self as bsp};
use usb_device::bus::UsbBusAllocator;

use crate::usb::Usb;

pub struct Board {
    led_pin: Pin<Gpio25, FunctionSio<SioOutput>, PullDown>,
    relay_pin: Pin<Gpio2, FunctionSio<SioOutput>, PullDown>,
    pub soil_sensor_address: Option<u8>,
    pub i2c: I2C<
        I2C0,
        (
            Pin<Gpio4, FunctionI2c, PullUp>,
            Pin<Gpio5, FunctionI2c, PullUp>,
        ),
    >,
    pub delay: cortex_m::delay::Delay,
    pub usb: &'static mut Usb,
}

static mut USB_BUS_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_INSTACE: Option<&mut Usb> = None;

impl Board {
    pub fn new() -> Board {
        let mut pac = pac::Peripherals::take().unwrap();
        let core = pac::CorePeripherals::take().unwrap();

        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let sio = Sio::new(pac.SIO);

        let external_xtal_freq = 12_000_000u32;
        // let mut resets = Board::get_pac().RESETS;
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

        let pins = bsp::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        let i2c_serial_freq = 100_000u32;
        let mut i2c = I2C::i2c0(
            pac.I2C0,
            pins.gpio4.reconfigure(),
            pins.gpio5.reconfigure(),
            i2c_serial_freq.Hz(),
            &mut pac.RESETS,
            clocks.system_clock.freq(),
        );

        let soil_sensor_address = Board::find_i2c_device_address(&mut i2c);

        let usb_bus_allocator = unsafe {
            USB_BUS_ALLOCATOR = Some(UsbBusAllocator::new(UsbBus::new(
                pac.USBCTRL_REGS,
                pac.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut pac.RESETS,
            )));
            USB_BUS_ALLOCATOR.as_mut().unwrap()
        };
        let usb = unsafe {
            USB_INSTACE = Some(Usb::new(usb_bus_allocator));
            USB_INSTACE.as_mut().unwrap()
        };

        Self {
            led_pin: pins.led.into_push_pull_output(),
            relay_pin: pins
                .gpio2
                .into_push_pull_output_in_state(embedded_hal::digital::PinState::High),
            soil_sensor_address,
            i2c,
            delay: cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz()),
            usb: *usb,
        }
    }

    pub fn find_i2c_device_address(
        i2c: &mut I2C<
            I2C0,
            (
                Pin<Gpio4, FunctionI2c, PullUp>,
                Pin<Gpio5, FunctionI2c, PullUp>,
            ),
        >,
    ) -> Option<u8> {
        for i in 0..=127u8 {
            let mut readbuf: [u8; 1] = [0; 1];
            let result = embedded_hal::i2c::I2c::read(i2c, i, &mut readbuf);
            if result.is_ok() {
                return Some(i);
            }
        }

        None
    }

    pub fn found_soil_sensor(&self) -> bool {
        return self.soil_sensor_address.is_some();
    }

    pub fn read_moisture(&mut self) -> u32 {
        let Some(address) = self.soil_sensor_address else {
            return 0;
        };
        let module_base_address = 0x0Fu8;
        let moisture_register_address = 0x10u8;
        let writebuf = [module_base_address, moisture_register_address];
        embedded_hal::i2c::I2c::write(&mut self.i2c, address, &writebuf).unwrap();

        self.delay.delay_ms(5);

        let mut readbuf: [u8; 2] = [0; 2];
        embedded_hal::i2c::I2c::read(&mut self.i2c, address, &mut readbuf).unwrap();
        (readbuf[0] as u32) * readbuf[1] as u32
    }

    pub fn read_temperature(&mut self) -> f32 {
        let Some(address) = self.soil_sensor_address else {
            return 0f32;
        };
        let status_base_address = 0x00u8;
        let temperature_register_address = 0x04u8;
        let writebuf = [status_base_address, temperature_register_address];

        embedded_hal::i2c::I2c::write(&mut self.i2c, address, &writebuf).unwrap();

        self.delay.delay_ms(5);

        let mut readbuf: [u8; 4] = [0; 4];
        embedded_hal::i2c::I2c::read(&mut self.i2c, address, &mut readbuf).unwrap();
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

    pub fn set_led(&mut self, on: bool) {
        match on {
            true => self.led_pin.set_high(),
            false => self.led_pin.set_low(),
        }
        .unwrap();
    }

    pub fn set_relay(&mut self, open: bool) {
        match open {
            true => self.relay_pin.set_low(),
            false => self.relay_pin.set_high(),
        }
        .unwrap();
    }

    pub fn get_relay_open(&mut self) -> bool {
        self.relay_pin.is_set_low().is_ok()
    }
}
