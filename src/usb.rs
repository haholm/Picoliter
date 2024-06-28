use rp_pico::{
    hal::{self, clocks::UsbClock},
    pac::{RESETS, USBCTRL_DPRAM, USBCTRL_REGS},
};

use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{DefaultBufferStore, SerialPort};

static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

pub struct Usb {
    pub serial: SerialPort<'static, hal::usb::UsbBus, DefaultBufferStore, DefaultBufferStore>,
    pub usb_device: UsbDevice<'static, hal::usb::UsbBus>,
}

impl Usb {
    pub fn new(
        ctrl_reg: USBCTRL_REGS,
        ctrl_dpram: USBCTRL_DPRAM,
        pll: UsbClock,
        force_vbus_detect_bit: bool,
        resets: &mut RESETS,
    ) -> Self {
        let usb_bus = unsafe {
            USB_BUS = Some(UsbBusAllocator::new(hal::usb::UsbBus::new(
                ctrl_reg,
                ctrl_dpram,
                pll,
                force_vbus_detect_bit,
                resets,
            )));
            USB_BUS.as_ref().unwrap()
        };

        let serial = SerialPort::new(usb_bus);
        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("Raspberry Pi")
                .product("Pico")
                .serial_number("0")])
            .unwrap()
            .device_class(2)
            .build();

        Self { serial, usb_device }
    }
}

impl core::fmt::Write for Usb {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.serial.write(s.as_bytes()).unwrap();

        Ok(())
    }
}
