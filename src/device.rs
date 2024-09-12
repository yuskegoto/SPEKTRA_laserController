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
    ENCODER_RATIO, GPIO_SLEEP_DURATION, LED_BRIGHTNESS, MSG_BUF_DOWNSTREAM, MSG_BUF_UPSTREAM,
    SOFTWARE_VERSION,
};

const AS5600_ADDRESS: u8 = 0x36;
const AS5600_ANGLE_REGISTER: u8 = 0x0E;

const ENCODER_MAX_READING: u16 = 0xFFF;
// const SOLENOID_BOOST_DURATION_MS: u16 = 2000u16; //1500u16; //750u16;
const SOLENOID_ACTIVATE_DUTY_PPERCENT: u32 = 40u32;
const SOLENOID_ACTIVE_MAX_DURATION_MS: u16 = 5000u16;

// #[derive(FromPrimitive, Clone, Copy, PartialEq)]
// pub enum MachineState {
//     Halt = 0,
//     Forward = 1,
//     Backward = 2,
//     Error = 100,
// }

struct DeviceState {
    // device_state: MachineState,
    color_r: f32,
    color_b: f32,
    color_g: f32,
}
// struct ActionState {
//     brake_release_delay_ms: u16,
//     brake_release_delay_us: u32,
//     hand_grab_delay_ms: u16,
//     hand_grab_delay_us: u32,
//     hand_throw_delay_ms: u16,
//     catch_action_started: bool,
//     catch_action_us_started: bool,
//     throw_action_started: bool,
//     catch_timestamp_us: u128,
//     throw_timestamp_us: u128,

//     // Hand solenoid timer
//     solenoid_boost: bool,
//     solenoid_timestamp_us: u128,
//     solenoid_boot_duration_ms: u16,
// }

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
    // pin_brake: PinDriver<'a, V, Output>,
    solenoid_activate_duty: u32,
    ts: SystemTime,
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

        let solenoid_activate_duty =
            channel_laser_red.get_max_duty() * SOLENOID_ACTIVATE_DUTY_PPERCENT / 100;
        let ts: SystemTime = SystemTime::now();

        Self {
            sender,
            receiver,
            state,
            // action,
            ws2812,
            channel_laser_red,
            // pin_brake,
            solenoid_activate_duty,
            ts,
        }
    }

    pub fn init(&mut self) -> Result<()> {
        self.set_laser_color(0.0, 0.0, 0.0)?;
        self.update_led(0u8, 5u8, 0u8)?;

        Ok(())
    }

    // Main device update loop
    pub fn update(&mut self) -> Result<()> {
        // Read mag encoder
        // if (self.state.encoder_read_counter % SENSOR_READ_INTERVAL) == 0 {
        //     if let Ok((angle_data, speed_data)) = self.read_sensor() {
        //         let _ = self.send_upstream_message_float_2(Msg::AngleSpeed, angle_data, speed_data);
        //     };
        //     self.state.encoder_read_counter = 0;

        //     // info!(
        //     //     "sensing loop: {:?}us",
        //     //     self.ts.elapsed().unwrap().as_micros()
        //     // );
        //     // self.ts = SystemTime::now();
        // }
        // self.state.encoder_read_counter += 1;

        let ret = self.check_downstream_message();

        // If got action message, issue actuation command!
        if let Ok(msg) = ret {
            // if msg != Msg::None {
            //     info!("Msg:{}", msg as u8);
            // }
            match msg {
                Msg::ColorSet => {
                    let r: u8 = (self.state.color_r * LED_BRIGHTNESS as f32) as u8;
                    let g: u8 = (self.state.color_g * LED_BRIGHTNESS as f32) as u8;
                    let b: u8 = (self.state.color_b * LED_BRIGHTNESS as f32) as u8;
                    self.update_led(r, g, b)?;
                }
                _ => {}
            }
        }

        Ok(())
    }

    // Handle message from PC
    fn check_downstream_message(&mut self) -> Result<Msg> {
        if let Some(frame) = self.receiver.read() {
            info!("Downstream msg:{:x} Size:{}", frame[0], frame.len());
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

    // fn read_sensor(&mut self) -> Result<(f32, f32), EspError> {
    //     let write_data = [AS5600_ANGLE_REGISTER];
    //     let mut data = [0u8; 2];

    //     let res = self
    //         .i2c
    //         .write_read(AS5600_ADDRESS, &write_data, &mut data, 10);
    //     match res {
    //         Ok(_) => {
    //             let read_angle = u16::from_be_bytes(data);

    //             if self.state.angle_enc_raw > read_angle {
    //                 self.state.device_state = MachineState::Forward;
    //                 if self.state.angle_enc_raw - read_angle > 2048 {
    //                     self.state.encoder_turns += 1;
    //                     // info!("Roll H -> L: {}", self.state.encoder_turns);
    //                 }
    //             } else if self.state.angle_enc_raw < read_angle {
    //                 self.state.device_state = MachineState::Backward;
    //                 if read_angle - self.state.angle_enc_raw > 2048 {
    //                     self.state.encoder_turns -= 1;
    //                     // info!("Roll L -> H: {}", self.state.encoder_turns);
    //                 }
    //             }
    //             // Rollover enccoder counts
    //             if self.state.encoder_turns >= ENCODER_RATIO {
    //                 self.state.encoder_turns = 0;
    //             } else if self.state.encoder_turns < 0 {
    //                 self.state.encoder_turns = ENCODER_RATIO - 1;
    //             }

    //             // Get rod angle of the rod
    //             self.state.angle_rod_raw =
    //                 read_angle + ENCODER_MAX_READING * self.state.encoder_turns as u16;

    //             // Store current reading for the next evaluation
    //             self.state.angle_enc_raw = read_angle;

    //             // self.state.angle =
    //             let mut new_angle =
    //                 (self.state.angle_rod_raw as f32) * 360.0 / (4096.0 * ENCODER_RATIO as f32);
    //             // Invert angle for the cw
    //             new_angle = 360.0 - new_angle;
    //             self.state.angle_raw = new_angle;

    //             // Offset angle
    //             new_angle -= self.state.angle_offset;

    //             // Normalize angle
    //             if new_angle < 0.0 {
    //                 new_angle += 360.0;
    //             } else if new_angle >= 360.0 {
    //                 new_angle -= 360.0;
    //             }

    //             // Get angle speed
    //             let current_timestamp_us = EspSystemTime {}.now().as_micros();
    //             let time_diff = current_timestamp_us - self.state.read_timestamp_us;
    //             // if (self.state.loop_counter / 200) == 0 {
    //             //     self.state.loop_counter = 0;
    //             //     info!("{time_diff}");
    //             // }
    //             // self.state.loop_counter += 1;

    //             let angle_diff = (new_angle - self.state.angle).abs();
    //             let speed = if angle_diff < 128.0 {
    //                 (new_angle - self.state.angle) / (time_diff as f32 * 1e-6)
    //             } else if new_angle > self.state.angle {
    //                 // info!("angle roll up {}, {}", new_angle, self.state.angle);
    //                 (new_angle - (self.state.angle + 360.0)) / (time_diff as f32 * 1e-6)
    //             } else {
    //                 // info!("angle roll down {}, {}", new_angle, self.state.angle);
    //                 (new_angle - (self.state.angle - 360.0)) / (time_diff as f32 * 1e-6)
    //             };

    //             self.state.read_timestamp_us = current_timestamp_us;
    //             self.state.angle = new_angle;

    //             // Smooth speed
    //             self.state.speed *= 0.9;
    //             self.state.speed += 0.1 * speed;

    //             // Surpress initial spike
    //             let ts = EspSystemTime {}.now().as_millis();
    //             if self.state.speed.abs() > 2000.0 && ts < 5000 {
    //                 self.state.speed = 0.0;
    //             }

    //             // info!("Angle: {}, Speed:{}", new_angle, self.state.speed);

    //             // if cfg!(feature = "test") {
    //             //     info!(
    //             //         "AS5600 read: {:x?} raw:{}, angle:{}",
    //             //         data, self.state.angle_enc_raw, self.state.angle
    //             //     );
    //             // }
    //             Ok((self.state.angle, self.state.speed))
    //         }
    //         Err(e) => {
    //             error!("I2C read error: {e}");
    //             self.state.device_state = MachineState::Error;
    //             Err(e)
    //         }
    //     }
    // }

    fn set_laser_color(&mut self, red: f32, green: f32, blue: f32) -> Result<()> {
        let res = self
            .channel_laser_red
            .set_duty(self.channel_laser_red.get_max_duty())
            .unwrap();

        Ok(())
    }

    /**
     * Send angle data to upstream
     */
    fn send_upstream_message(&mut self, angle: &[u8; 2]) -> Result<()> {
        // info!("Sending angle info Angle:{:x?}", angle);

        if let Ok(mut wg) = self.sender.grant(2) {
            wg.to_commit(2);
            wg.copy_from_slice(angle);
            wg.commit(2);
        }
        Ok(())
    }

    fn send_upstream_message_ip(&mut self, ip: &[u8]) -> Result<()> {
        info!("Sending Ip Msg: len:{}", ip.len());

        let len = ip.len() + 1;
        if let Ok(mut wg) = self.sender.grant(len) {
            wg.to_commit(1);
            wg[0] = Msg::SetDestIp as u8;
            wg[1..].copy_from_slice(ip);
            wg.commit(len);
        }
        Ok(())
    }

    fn send_upstream_message_float(&mut self, msg: Msg, val: f32) -> Result<()> {
        let serialized = bincode::serialize(&val).unwrap();
        // if cfg!(feature = "test") {
        //     info!(
        //         "Sending read value Msg:{:x}, Val:{} {:x?}",
        //         msg as u8, val, serialized
        //     );
        // }
        let len = serialized.len() + 1;
        if let Ok(mut wg) = self.sender.grant(len) {
            wg.to_commit(1);
            wg[0] = msg as u8;
            wg[1..].copy_from_slice(&serialized);
            wg.commit(len);
        } else {
            error!("Failed to get upstream message buf");
        }
        Ok(())
    }

    fn send_upstream_message_float_2(&mut self, msg: Msg, val1: f32, val2: f32) -> Result<()> {
        let serialized1 = bincode::serialize(&val1).unwrap();
        let serialized2 = bincode::serialize(&val2).unwrap();

        let len = serialized1.len() + serialized2.len() + 1;
        if let Ok(mut wg) = self.sender.grant(len) {
            wg.to_commit(1);
            wg[0] = msg as u8;
            wg[1..5].copy_from_slice(&serialized1);
            wg[5..len].copy_from_slice(&serialized2);
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
