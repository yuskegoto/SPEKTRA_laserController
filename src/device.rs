use std::time::SystemTime;

use anyhow::Result;
use log::*;

use bbqueue::framed::{FrameConsumer, FrameProducer};
use esp_idf_hal::{delay, gpio::*, i2c::*, ledc};
use esp_idf_svc::systime::EspSystemTime;
use esp_idf_sys::EspError;
use smart_leds::RGB8;
use ws2812_esp32_rmt_driver::Ws2812Esp32Rmt;

use num_derive::FromPrimitive;
extern crate bincode;

use crate::Msg;
use crate::{
    LASER_BRIGHTNESS, LED_BRIGHTNESS, MSG_BUF_DOWNSTREAM, MSG_BUF_UPSTREAM, SOFTWARE_VERSION,
};

const AS5600_ADDRESS: u8 = 0x36;
const AS5600_ANGLE_REGISTER: u8 = 0x0E;

const ENCODER_MAX_READING: u16 = 0xFFF;
// const SOLENOID_BOOST_DURATION_MS: u16 = 2000u16; //1500u16; //750u16;
const SOLENOID_ACTIVATE_DUTY_PPERCENT: u32 = 40u32;
const SOLENOID_ACTIVE_MAX_DURATION_MS: u16 = 5000u16;

struct DeviceState {
    color_r: f32,
    color_b: f32,
    color_g: f32,
    // 0.0 - 1.0
    laser_brightness: f32,
}

pub struct Device<'a> {
    // pub struct Device<'a, V: Pin> {
    // pub struct Device<'a, U: Pin, V: Pin> {
    sender: FrameProducer<'static, MSG_BUF_UPSTREAM>,
    receiver: FrameConsumer<'static, MSG_BUF_DOWNSTREAM>,
    state: DeviceState,
    // action: ActionState,
    ws2812: Ws2812Esp32Rmt<'a>,
    // pin_solenoid: PinDriver<'a, U, Output>,
    channel_laser_red: ledc::LedcDriver<'a>,
    channel_laser_green: ledc::LedcDriver<'a>,
    channel_laser_blue: ledc::LedcDriver<'a>,
    // pin_brake: PinDriver<'a, V, Output>,
    // solenoid_activate_duty: u32,
    // ts: SystemTime,
}

impl<'a> Device<'a>
// impl<'a, V> Device<'a, V>
// impl<'a, U, V> Device<'a, U, V>
where
// U: Pin,
// V: Pin,
{
    pub fn new(
        sender: FrameProducer<'static, MSG_BUF_UPSTREAM>,
        receiver: FrameConsumer<'static, MSG_BUF_DOWNSTREAM>,
        ws2812: Ws2812Esp32Rmt<'a>,
        // pin_solenoid: PinDriver<'a, U, Output>,
        channel_laser_red: ledc::LedcDriver<'a>,
        channel_laser_green: ledc::LedcDriver<'a>,
        channel_laser_blue: ledc::LedcDriver<'a>,
        // pin_brake: PinDriver<'a, V, Output>,
    ) -> Self {
        let state = DeviceState {
            // device_state: MachineState::Halt,
            // speed: 0f32,
            // angle: 0f32,
            // angle_offset: 0f32,
            // angle_raw: 0f32,
            // angle_enc_raw: 0u16,
            // angle_rod_raw: 0u16,
            // encoder_turns: 0i8,
            // read_timestamp_us: 0u128,
            // brake_hold: false,
            // hand_closed: false,
            // encoder_read_counter: 0u8,
            // loop_counter: 0u16,
            color_r: 0f32,
            color_b: 0f32,
            color_g: 0f32,
            laser_brightness: LASER_BRIGHTNESS,
        };

        // let action = ActionState {
        //     brake_release_delay_ms: 0u16,
        //     brake_release_delay_us: 0u32,
        //     hand_grab_delay_ms: 0u16,
        //     hand_grab_delay_us: 0u32,
        //     hand_throw_delay_ms: 0u16,
        //     catch_action_started: false,
        //     catch_action_us_started: false,
        //     throw_action_started: false,

        //     catch_timestamp_us: 0u128,
        //     throw_timestamp_us: 0u128,

        //     solenoid_boost: false,
        //     solenoid_timestamp_us: 0u128,
        //     solenoid_boot_duration_ms: 1000u16,
        // };

        // let solenoid_activate_duty =
        //     channel_laser_red.get_max_duty() * SOLENOID_ACTIVATE_DUTY_PPERCENT / 100;
        // let ts: SystemTime = SystemTime::now();

        Self {
            sender,
            receiver,
            state,
            // action,
            ws2812,
            channel_laser_red,
            channel_laser_green,
            channel_laser_blue,
            // solenoid_activate_duty,
            // ts,
        }
    }

    pub fn init(&mut self) -> Result<()> {
        self.set_laser_color(0.5, 0.5, 0.5)?;
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
        let r: u8 = (self.state.color_r * LED_BRIGHTNESS as f32) as u8;
        let g: u8 = (self.state.color_g * LED_BRIGHTNESS as f32) as u8;
        let b: u8 = (self.state.color_b * LED_BRIGHTNESS as f32) as u8;
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
                    if frame.len() == 13 {
                        self.state.color_r = bincode::deserialize(&frame[1..5]).unwrap();
                        self.state.color_g = bincode::deserialize(&frame[5..9]).unwrap();
                        self.state.color_b = bincode::deserialize(&frame[9..13]).unwrap();

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

        //     match self.state.device_state {
        // MachineState::Halt => {
        // }
        // MachineState::Forward => {
        //     self.set_led_color(RGB8 {
        //         r: 0,
        //         g: LED_BRIGHTNESS,
        //         b: 0,
        //     })
        //     .unwrap();
        // }
        // MachineState::Backward => {
        //     self.set_led_color(RGB8 {
        //         r: 0,
        //         g: 0,
        //         b: LED_BRIGHTNESS,
        //     })
        //     .unwrap();
        // }
        // MachineState::Error => {
        //     self.set_led_color(RGB8 {
        //         r: LED_BRIGHTNESS,
        //         g: 0,
        //         b: 0,
        //     })
        //     .unwrap();
        // }
        // }

        Ok(())
    }

    fn set_led_color(&mut self, color: RGB8) -> Result<()> {
        let pixels = std::iter::repeat(color).take(1);
        if let Err(e) = self.ws2812.write_nocopy(pixels) {
            error!("WS2812 write error: {e}");
        };
        Ok(())
    }

    fn set_laser_color(&mut self, red: f32, green: f32, blue: f32) -> Result<()> {
        let r = (self.channel_laser_red.get_max_duty() as f32 * red * self.state.laser_brightness)
            as u32;
        let _ = self.channel_laser_red.set_duty(r);

        let g = (self.channel_laser_green.get_max_duty() as f32
            * green
            * self.state.laser_brightness) as u32;
        let _ = self.channel_laser_green.set_duty(g);

        let b = (self.channel_laser_blue.get_max_duty() as f32 * blue * self.state.laser_brightness)
            as u32;
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

    // fn send_upstream_message_float(&mut self, msg: Msg, val: f32) -> Result<()> {
    //     let serialized = bincode::serialize(&val).unwrap();
    //     // if cfg!(feature = "test") {
    //     //     info!(
    //     //         "Sending read value Msg:{:x}, Val:{} {:x?}",
    //     //         msg as u8, val, serialized
    //     //     );
    //     // }
    //     let len = serialized.len() + 1;
    //     if let Ok(mut wg) = self.sender.grant(len) {
    //         wg.to_commit(1);
    //         wg[0] = msg as u8;
    //         wg[1..].copy_from_slice(&serialized);
    //         wg.commit(len);
    //     } else {
    //         error!("Failed to get upstream message buf");
    //     }
    //     Ok(())
    // }

    // fn send_upstream_message_float_2(&mut self, msg: Msg, val1: f32, val2: f32) -> Result<()> {
    //     let serialized1 = bincode::serialize(&val1).unwrap();
    //     let serialized2 = bincode::serialize(&val2).unwrap();

    //     let len = serialized1.len() + serialized2.len() + 1;
    //     if let Ok(mut wg) = self.sender.grant(len) {
    //         wg.to_commit(1);
    //         wg[0] = msg as u8;
    //         wg[1..5].copy_from_slice(&serialized1);
    //         wg[5..len].copy_from_slice(&serialized2);
    //         wg.commit(len);
    //     }
    //     Ok(())
    // }

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
