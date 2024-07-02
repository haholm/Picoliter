#![no_std]
#![no_main]

use bsp::entry;
use core::fmt::Write as _;
use defmt_rtt as _;

use rp_pico::{self as bsp};

pub mod usb;

pub mod board;
use board::Board;

#[entry]
fn main() -> ! {
    let mut board = Board::new();

    board.delay.delay_ms(5000);

    board.soil_sensor_address = Board::find_i2c_device_address(&mut board.i2c);

    loop {
        if board.found_soil_sensor() {
            let moisture = board.read_moisture();
            let temperature = board.read_temperature();

            board.usb.log_clear(usb::LogClearModes::Line);
            board.usb.log_move(usb::LogMoveModes::DownLine);
            board.usb.log_move(usb::LogMoveModes::UpLine);
            write!(
                board.usb,
                "Moisture: {}, Temperature: {}°C.",
                moisture, temperature
            )
            .unwrap();

            if moisture > 500 {
                board.set_relay(true);
            } else {
                board.set_relay(false);
            }
        }

        board.set_led(true);
        board.delay.delay_ms(500);

        board.set_led(false);
        board.delay.delay_ms(500);
    }
}
