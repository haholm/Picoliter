use crate::bsp::hal::pac::{self, interrupt};
use core::{fmt::Write, panic::PanicInfo};
use rp_pico::hal::{self};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{DefaultBufferStore, SerialPort};

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
            usb_instance.log_move(LogMoveModes::DownLine);
            write!(usb_instance, "{}", info).unwrap();
        }
        _ => (),
    }

    loop {}
}

pub enum LogClearModes {
    Screen,
    Line,
}

pub enum LogMoveModes {
    UpLine,
    DownLine,
}

pub struct Usb {
    pub serial: SerialPort<'static, hal::usb::UsbBus, DefaultBufferStore, DefaultBufferStore>,
    pub usb_device: UsbDevice<'static, hal::usb::UsbBus>,
}

impl Usb {
    pub fn new(usb_bus_allocator: &'static UsbBusAllocator<hal::usb::UsbBus>) -> &'static mut Self {
        let serial = SerialPort::new(&usb_bus_allocator);
        let usb_device = UsbDeviceBuilder::new(&usb_bus_allocator, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("Raspberry Pi")
                .product("Pico")
                .serial_number("0")])
            .unwrap()
            .device_class(2)
            .build();

        unsafe {
            USB_INSTANCE = Some(Self { serial, usb_device });
            pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
            USB_INSTANCE.as_mut().unwrap()
        }
    }

    pub fn log_move(&mut self, mode: LogMoveModes) {
        let code = match mode {
            LogMoveModes::UpLine => "\x1B[F",
            LogMoveModes::DownLine => "\x1B[E",
        };
        self.write_str(code).unwrap();
    }

    pub fn log_clear(&mut self, mode: LogClearModes) {
        let code = match mode {
            LogClearModes::Screen => "\x1B[2J",
            LogClearModes::Line => "\x1B[2K",
        };
        self.write_str(code).unwrap();
    }
}

impl core::fmt::Write for Usb {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.serial.write(s.as_bytes()).unwrap();

        Ok(())
    }
}
