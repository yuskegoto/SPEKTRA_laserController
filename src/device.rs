use std::time::SystemTime;

use anyhow::Result;
use log::*;

use bbqueue::framed::{FrameConsumer, FrameProducer};
use esp_idf_hal::{delay, gpio::*, i2c::*, ledc};
// use esp_idf_svc::systime::EspSystemTime;
// use esp_idf_sys::EspError;
use smart_leds::RGB8;
use ws2812_esp32_rmt_driver::Ws2812Esp32Rmt;

// use num_derive::FromPrimitive;
extern crate bincode;

use crate::Msg;
use crate::{
    LASER_BRIGHTNESS, LED_BRIGHTNESS, MSG_BUF_DOWNSTREAM, MSG_BUF_UPSTREAM, SOFTWARE_VERSION,
};

pub const LASER_MAX_DUTY: u8 = 255;

struct DeviceState {
    color_r: u8,
    color_b: u8,
    color_g: u8,
    // 0.0 - 1.0
    laser_brightness: f32,
}

pub struct Device<'a> {
    sender: FrameProducer<'static, MSG_BUF_UPSTREAM>,
    receiver: FrameConsumer<'static, MSG_BUF_DOWNSTREAM>,
    state: DeviceState,
    ws2812: Ws2812Esp32Rmt<'a>,
    channel_laser_red: ledc::LedcDriver<'a>,
    channel_laser_green: ledc::LedcDriver<'a>,
    channel_laser_blue: ledc::LedcDriver<'a>,
}

impl<'a> Device<'a> {
    pub fn new(
        sender: FrameProducer<'static, MSG_BUF_UPSTREAM>,
        receiver: FrameConsumer<'static, MSG_BUF_DOWNSTREAM>,
        ws2812: Ws2812Esp32Rmt<'a>,
        channel_laser_red: ledc::LedcDriver<'a>,
        channel_laser_green: ledc::LedcDriver<'a>,
        channel_laser_blue: ledc::LedcDriver<'a>,
    ) -> Self {
        let state = DeviceState {
            color_r: 0u8,
            color_b: 0u8,
            color_g: 0u8,
            laser_brightness: LASER_BRIGHTNESS,
        };

        Self {
            sender,
            receiver,
            state,
            ws2812,
            channel_laser_red,
            channel_laser_green,
            channel_laser_blue,
        }
    }

    pub fn init(&mut self) -> Result<()> {
        self.set_laser_color(100, 100, 100)?;
        self.update_led(0u8, 5u8, 0u8)?;

        Ok(())
    }

    // Main device update loop
    pub fn update(&mut self) -> Result<()> {
        let ret = self.check_downstream_message();

        // If got action message, issue actuation command!
        if let Ok(msg) = ret {
            // if msg != Msg::None {
            //     info!("Msg:{}", msg as u8);
            // }
            match msg {
                Msg::ColorSet | Msg::MaxDutySet => {
                    self.update_led_laser_colors()?;
                }
                _ => {}
            }
        } else {
            self.update_led(0, 0, 0)?;
        }

        Ok(())
    }

    fn update_led_laser_colors(&mut self) -> Result<()> {
        let r: u8 = self.state.color_r / LED_BRIGHTNESS;
        let g: u8 = self.state.color_g / LED_BRIGHTNESS;
        let b: u8 = self.state.color_b / LED_BRIGHTNESS;
        self.update_led(r, g, b)?;
        self.set_laser_color(self.state.color_r, self.state.color_g, self.state.color_b)?;
        // info!("Color set: R:{}, G:{}, B:{}", r, g, b);
        Ok(())
    }

    // Handle message from PC
    fn check_downstream_message(&mut self) -> Result<Msg> {
        if let Some(frame) = self.receiver.read() {
            // info!("Downstream msg:{:x} Size:{}", frame[0], frame.len());
            let msg_type: Option<Msg> = num::FromPrimitive::from_u8(frame[0]);

            let ret_msg = match msg_type {
                Some(Msg::Version) => {
                    self.send_upstream_message_u8(Msg::Version, SOFTWARE_VERSION)
                        .unwrap();
                    Msg::Version
                }
                Some(Msg::ColorSet) => {
                    if frame.len() == 4 {
                        self.state.color_r = frame[1];
                        self.state.color_g = frame[2];
                        self.state.color_b = frame[3];

                        Msg::ColorSet
                    } else {
                        Msg::None
                    }
                }

                Some(Msg::MaxDutySet) => {
                    if frame.len() == 5 {
                        let max_duty: f32 = bincode::deserialize(&frame[1..5]).unwrap();
                        self.state.laser_brightness = max_duty.clamp(0.0, 1.0);

                        // info!("Max Duty Set:{}", max_duty);
                        Msg::MaxDutySet
                    } else {
                        Msg::None
                    }
                }

                Some(Msg::SetDestIp) => {
                    if frame.len() == 5 {
                        info!("Set dest IP{:>?}", &frame[1..]);
                        self.send_upstream_message_ip(&frame[1..5]).unwrap();
                    }
                    Msg::None
                }

                _ => Msg::None,
            };

            frame.release();
            return Ok(ret_msg);
        }
        Ok(Msg::None)
    }

    fn update_led(&mut self, red: u8, green: u8, blue: u8) -> Result<()> {
        self.set_led_color(RGB8 {
            r: red,
            g: green,
            b: blue,
        })
        .unwrap();

        Ok(())
    }

    fn set_led_color(&mut self, color: RGB8) -> Result<()> {
        let pixels = std::iter::repeat(color).take(1);
        if let Err(e) = self.ws2812.write_nocopy(pixels) {
            error!("WS2812 write error: {e}");
        };
        Ok(())
    }

    fn set_laser_color(&mut self, red: u8, green: u8, blue: u8) -> Result<()> {
        let r = (red as f32 * self.state.laser_brightness) as u32;
        let _ = self.channel_laser_red.set_duty(r);

        let g = (green as f32 * self.state.laser_brightness) as u32;
        let _ = self.channel_laser_green.set_duty(g);

        let b = (blue as f32 * self.state.laser_brightness) as u32;
        let _ = self.channel_laser_blue.set_duty(b);

        Ok(())
    }

    fn send_upstream_message_ip(&mut self, ip: &[u8]) -> Result<()> {
        // info!("Sending Ip Msg: len:{}", ip.len());

        let len = ip.len() + 1;
        if let Ok(mut wg) = self.sender.grant(len) {
            wg.to_commit(1);
            wg[0] = Msg::SetDestIp as u8;
            wg[1..].copy_from_slice(ip);
            wg.commit(len);
        }
        Ok(())
    }

    fn send_upstream_message_u8(&mut self, msg: Msg, val: u8) -> Result<()> {
        info!("Sending info Msg:{:x}, Val:{:x}", msg as u8, val);

        if let Ok(mut wg) = self.sender.grant(2) {
            wg.to_commit(1);
            wg[0] = msg as u8;
            wg[1] = val;
            wg.commit(2);
        }
        Ok(())
    }
}
