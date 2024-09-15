use anyhow::Result;
use esp_idf_hal::io::Write;
use log::*;
use num_traits::ToBytes;
use std::time::{Duration, SystemTime};

use esp_idf_hal::{
    delay,
    gpio::*,
    task::*,
    uart::{AsyncUartDriver, Uart, UartDriver},
};
use esp_idf_svc::systime::EspSystemTime;
use esp_idf_sys::EspError;

use bbqueue::framed::{FrameConsumer, FrameProducer};
use crc16::*;

use num_derive::FromPrimitive;
extern crate bincode;

use crate::Msg;

use crate::{
    ANGLE_REPORT_INTERVAL_MS, DYNAMIXEL_TASK_INTERVAL_MS, MSG_DXL_BUF_DOWNSTREAM,
    MSG_DXL_BUF_UPSTREAM,
};

const DYNAMIXEL_RESOLUTION: f32 = 4096.0;
const PITCH_LIMIT_ANGLE: f32 = 91.0;
const YAW_LIMIT_ANGLE: f32 = 181.0;

const DYNAMIXEL_PACKET_HEADER: [u8; 4] = [0xFFu8, 0xFFu8, 0xFDu8, 0x00u8];

const RESPONSE_BUF_SIZE: usize = 15;

#[allow(dead_code)]
#[derive(FromPrimitive, Clone, Copy, PartialEq)]
enum DxlInstruction {
    Ping = 1,
    Read = 2,
    Write = 3,
}

enum DxlAddress {
    GoalPosition = 0x74,
}

#[derive(FromPrimitive, Clone, Copy, PartialEq)]
enum Motor {
    Yaw = 1,
    Pitch = 2,
}

pub struct Dynamixel<'a, V: Pin> {
    sender: FrameProducer<'static, MSG_DXL_BUF_UPSTREAM>,
    receiver: FrameConsumer<'static, MSG_DXL_BUF_DOWNSTREAM>,
    // dxl: UartDriver<'a>,
    dxl: AsyncUartDriver<'a, UartDriver<'a>>,
    dxl_en: PinDriver<'a, V, Output>,
    pitch_angle_f: f32,
    yaw_angle_f: f32,
    pitch_angle: i32,
    yaw_angle: i32,
    angle_report_timestamp: SystemTime,
    response_buf: [u8; RESPONSE_BUF_SIZE],
}

impl<'a, V> Dynamixel<'a, V>
where
    V: Pin,
{
    pub fn new(
        sender: FrameProducer<'static, MSG_DXL_BUF_UPSTREAM>,
        receiver: FrameConsumer<'static, MSG_DXL_BUF_DOWNSTREAM>,
        // dxl: UartDriver<'a>,
        dxl: AsyncUartDriver<'a, UartDriver<'a>>,
        dxl_en: PinDriver<'a, V, Output>,
    ) -> Self {
        let angle_report_timestamp = std::time::SystemTime::now();
        let response_buf = [0u8; RESPONSE_BUF_SIZE];
        Self {
            sender,
            receiver,
            dxl,
            dxl_en,
            pitch_angle_f: 0.0,
            pitch_angle: 0,
            yaw_angle_f: 0.0,
            yaw_angle: 0,
            angle_report_timestamp,
            response_buf,
        }
    }

    pub fn init(&mut self) -> Result<()> {
        self.ping(Motor::Yaw)?;
        self.ping(Motor::Pitch)?;
        self.read_angle(Motor::Yaw)?;
        self.set_led(Motor::Yaw, true)?;
        self.set_led(Motor::Pitch, true)?;
        self.enable_torque(Motor::Yaw, true)?;
        self.enable_torque(Motor::Pitch, true)?;

        Ok(())
    }

    // Dynamixel main loop
    pub fn update(&mut self) -> Result<()> {
        // Check downstream message over OSC
        let ret = self.check_downstream_message();

        // If got action message, issue actuation command!
        if let Ok(msg) = ret {
            // if msg != Msg::None {
            //     info!("Msg:{}", msg as u8);
            // }
            match msg {
                Msg::AngleSet => {
                    self.set_angle(Motor::Yaw, self.yaw_angle)?;
                    self.set_angle(Motor::Pitch, self.pitch_angle)?;
                }
                _ => {}
            }
            // } else {
            //     None
        };
        if self.angle_report_timestamp.elapsed().unwrap().as_millis() > ANGLE_REPORT_INTERVAL_MS {
            self.angle_report_timestamp = SystemTime::now();
            if let Ok(angle) = self.read_angle(Motor::Yaw) {
                self.yaw_angle = angle;
                self.yaw_angle_f = ((angle as f32 / DYNAMIXEL_RESOLUTION) * 360.0) - 180.0;
                // info!("Yaw Angle:{}", self.pitch_angle_f);
            }
            if let Ok(angle) = self.read_angle(Motor::Pitch) {
                self.pitch_angle = angle;
                self.pitch_angle_f = ((angle as f32 / DYNAMIXEL_RESOLUTION) * 360.0) - 180.0;
                // info!("Pitch Angle:{}", self.yaw_angle_f);
            }

            self.send_upstream_message_float_2(
                Msg::AngleReport,
                self.yaw_angle_f,
                self.pitch_angle_f,
            )?;
        }

        // If msg is available, send via Dynamixel

        Ok(())
    }

    /**
     * Sleep until next interval
     */
    pub fn idle(&self) {
        std::thread::sleep(DYNAMIXEL_TASK_INTERVAL_MS);
    }

    // fn write_dxl_message(&mut self, msg_buf: &[u8], response_buf: &mut [u8; 8]) -> Result<()> {
    //     // Write message
    //     let mut res_buf: [u8; 14] = [0u8; 14];

    //     block_on(async {
    //         self.dxl_en.set_high().unwrap();
    //         while !self.dxl_en.is_set_high() {}

    //         // let res = self.dxl.write(msg_buf);
    //         let res = self.dxl.write(msg_buf).await;
    //         match res {
    //             Ok(_) => {
    //                 // let r = self.dxl.flush();

    //                 let res = self.dxl.wait_tx_done().await;
    //                 match res {
    //                     Ok(_) => {
    //                         self.dxl_en.set_low().unwrap();
    //                         while !self.dxl_en.is_set_low() {}
    //                         // wait for answer
    //                         // block_on(async {
    //                         let res = self.dxl.read(&mut res_buf).await;
    //                         match res {
    //                             Ok(_) => {
    //                                 info!("Dynamixel read success: {:x?}", res_buf);
    //                             }
    //                             Err(e) => {
    //                                 error!("Dynamixel read failed: {:?}", e);
    //                             }
    //                         }
    //                         // });
    //                     }
    //                     Err(e) => {
    //                         error!("DXL TX failed: {:?}", e);
    //                     }
    //                 }
    //                 // delay::Ets::delay_us(1000);
    //             }
    //             Err(e) => {
    //                 error!("DXL packet write failed: {:?}", e);
    //             }
    //         }

    //         // delay::Ets::delay_ms(2);

    //         self.dxl_en.set_low().unwrap();
    //     });

    //     // std::thread::sleep(Duration::from_millis(1));
    //     Ok(())
    // }

    /**
     * Read length must be shorter than response_buf length
     * Actual read length should be set with read_length
     */
    fn read_write_dxl_message(
        &mut self,
        msg_buf: &[u8],
        // response_buf: &mut [u8; 15],
        read_length: usize,
    ) -> Result<()> {
        // Write message

        block_on(async {
            self.dxl_en.set_high().unwrap();
            while !self.dxl_en.is_set_high() {}

            // let res = self.dxl.write(msg_buf);
            let res = self.dxl.write(msg_buf).await;
            match res {
                Ok(_) => {
                    // let r = self.dxl.flush();

                    let res = self.dxl.wait_tx_done().await;
                    match res {
                        Ok(_) => {
                            self.dxl_en.set_low().unwrap();
                            while !self.dxl_en.is_set_low() {}
                            // wait for answer
                            self.response_buf.fill(0);
                            let res = self.dxl.read(&mut self.response_buf[..read_length]).await;
                            match res {
                                Ok(_) => {
                                    // info!("Dynamixel read success: {:x?}", response_buf);
                                }
                                Err(e) => {
                                    error!("Dynamixel read failed: {:?}", e);
                                }
                            }
                        }
                        Err(e) => {
                            error!("DXL TX failed: {:?}", e);
                        }
                    }
                    // delay::Ets::delay_us(1000);
                }
                Err(e) => {
                    error!("DXL packet write failed: {:?}", e);
                }
            }
            // delay::Ets::delay_ms(2);

            self.dxl_en.set_low().unwrap();
        });

        Ok(())
    }

    fn ping(&mut self, id: Motor) -> Result<()> {
        let mut msg_buf = vec![0xFFu8, 0xFFu8, 0xFDu8, 0x00u8];
        // ID
        msg_buf.push(id as u8);
        // msg_buf.push(0xFEu8);
        // msg_buf.push(0x01u8);
        // Length
        msg_buf.push(0x03u8);
        msg_buf.push(0x00u8);
        // Instruction
        msg_buf.push(0x01u8);

        // Get CRC
        let mut crc_state = State::<BUYPASS>::new();
        crc_state.update(&msg_buf);
        let crc_res = crc_state.get();
        info!("CRC: {:x?}", crc_res);
        msg_buf.extend_from_slice(&crc_res.to_le_bytes());

        // CRC
        // msg_buf.push(0x19u8);
        // msg_buf.push(0x4Eu8);

        info!("Ping Msg: {:02X?}", msg_buf);

        // let mut response_buf: [u8; 8] = [0u8; 8];
        self.response_buf.fill(0);
        self.read_write_dxl_message(&msg_buf, 8)?;
        // self.write_dxl_message(&msg_buf, &mut self.response_buf)?;

        Ok(())
    }

    fn read_angle(&mut self, id: Motor) -> Result<i32> {
        let mut msg_buf = vec![0xFFu8, 0xFFu8, 0xFDu8, 0x00u8];
        // ID
        msg_buf.push(id as u8);
        // Length
        msg_buf.push(0x07u8);
        msg_buf.push(0x00u8);
        // Instruction: Read
        msg_buf.push(0x02u8);

        // Position 132
        msg_buf.push(0x84u8);
        msg_buf.push(0x00u8);
        // Length 4
        msg_buf.push(0x04u8);
        msg_buf.push(0x00u8);

        // Get CRC
        // Should be fixed 0x15 0x1D
        let mut crc_state = State::<BUYPASS>::new();
        crc_state.update(&msg_buf);
        let crc_res = crc_state.get();
        // info!("CRC: {:x?}", crc_res);
        msg_buf.extend_from_slice(&crc_res.to_le_bytes());

        // info!("Position Msg: {:02X?}", msg_buf);

        // let mut response_buf: [u8; 15] = [0u8; 15];
        // let bufsize = response_buf.len();
        let position = if let Ok(_) = self.read_write_dxl_message(&msg_buf, 15) {
            i32::from_le_bytes(self.response_buf[9..13].try_into().unwrap())
        } else {
            0i32
        };
        // self.read_dxl_message(&msg_buf, &mut response_buf, bufsize)?;
        // info!("Dynamixel answer: {:x?}", response_buf);

        // let position = i32::from_le_bytes(response_buf[9..13].try_into().unwrap());
        // info!("Position:{}", position);

        Ok(position)
    }

    /**
     * LED control
     * Address 0x41 (65)
     */
    fn set_led(&mut self, id: Motor, led_on: bool) -> Result<()> {
        let mut msg_buf = DYNAMIXEL_PACKET_HEADER.to_vec();
        // let mut msg_buf = vec![0xFFu8, 0xFFu8, 0xFDu8, 0x00u8];
        // ID
        msg_buf.push(id as u8);
        // Length
        msg_buf.push(0x06u8);
        msg_buf.push(0x00u8);
        // Instruction: Write
        msg_buf.push(0x03u8);

        // LED control 0x41
        msg_buf.push(0x41u8);
        msg_buf.push(0x00u8);

        // data
        msg_buf.push(led_on as u8);

        // Get CRC
        let mut crc_state = State::<BUYPASS>::new();
        crc_state.update(&msg_buf);
        let crc_res = crc_state.get();
        info!("CRC: {:x?}", crc_res);
        msg_buf.extend_from_slice(&crc_res.to_le_bytes());

        info!("LED Msg: {:02X?}", msg_buf);

        // let mut response_buf: [u8; 8] = [0u8; 8];
        // self.write_dxl_message(&msg_buf, &mut response_buf)?;
        self.read_write_dxl_message(&msg_buf, 8)?;

        Ok(())
    }

    /**
     * Turn on / off torque control
     * Address 0x40 (64)
     */
    fn enable_torque(&mut self, id: Motor, enable: bool) -> Result<()> {
        let mut msg_buf = vec![0xFFu8, 0xFFu8, 0xFDu8, 0x00u8];
        // ID
        msg_buf.push(id as u8);
        // Length
        msg_buf.push(0x06u8);
        msg_buf.push(0x00u8);
        // Instruction: Write
        msg_buf.push(0x03u8);

        // Torque control 0x40
        msg_buf.push(0x40u8);
        msg_buf.push(0x00u8);

        // data
        msg_buf.push(enable as u8);

        // Get CRC
        let mut crc_state = State::<BUYPASS>::new();
        crc_state.update(&msg_buf);
        let crc_res = crc_state.get();
        // info!("CRC: {:x?}", crc_res);
        msg_buf.extend_from_slice(&crc_res.to_le_bytes());

        // info!("Torque Msg: {:02X?}", msg_buf);

        // let mut response_buf: [u8; 15] = [0u8; 15];
        // let bufsize = 11;
        // self.read_dxl_message(&msg_buf, &mut response_buf, bufsize)?;
        // info!("Dynamixel read success: {:x?}", response_buf);
        self.read_write_dxl_message(&msg_buf, 11)?;

        Ok(())
    }

    fn set_angle(&mut self, id: Motor, angle: i32) -> Result<()> {
        let mut msg_buf = DYNAMIXEL_PACKET_HEADER.to_vec();
        msg_buf.push(id as u8);
        // Length
        msg_buf.push(0x09u8);
        msg_buf.push(0x0u8);
        // Instruction: Write
        msg_buf.push(DxlInstruction::Write as u8);
        // Address
        msg_buf.push(DxlAddress::GoalPosition as u8);
        msg_buf.push(0x0u8);
        // Position
        msg_buf.extend_from_slice(&angle.to_le_bytes());

        // Get CRC
        let mut crc_state = State::<BUYPASS>::new();
        crc_state.update(&msg_buf);
        let crc_res = crc_state.get();
        msg_buf.extend_from_slice(&crc_res.to_le_bytes());

        // info!("Set Angle Msg: {} {:02X?}", angle, msg_buf);
        self.read_write_dxl_message(&msg_buf, 11)?;

        Ok(())
    }

    // Handle message from OSC downstream
    fn check_downstream_message(&mut self) -> Result<Msg> {
        if let Some(frame) = self.receiver.read() {
            // info!("Dynamixel downstream msg:{:x} Size:{}", frame[0], frame.len());
            let msg_type: Option<Msg> = num::FromPrimitive::from_u8(frame[0]);

            let ret_msg = match msg_type {
                Some(Msg::AngleSet) => {
                    if frame.len() == 9 {
                        let yaw: f32 = bincode::deserialize(&frame[1..5]).unwrap();
                        self.yaw_angle_f = yaw.clamp(-YAW_LIMIT_ANGLE, YAW_LIMIT_ANGLE);
                        self.yaw_angle = (self.yaw_angle_f / 360.0 * DYNAMIXEL_RESOLUTION) as i32;
                        self.yaw_angle += DYNAMIXEL_RESOLUTION as i32 / 2;

                        let pitch: f32 = bincode::deserialize(&frame[5..9]).unwrap();
                        self.pitch_angle_f = pitch.clamp(-PITCH_LIMIT_ANGLE, PITCH_LIMIT_ANGLE);
                        self.pitch_angle =
                            (self.pitch_angle_f / 360.0 * DYNAMIXEL_RESOLUTION) as i32;
                        self.pitch_angle += DYNAMIXEL_RESOLUTION as i32 / 2;

                        info!(
                            "Yaw:{}, {} Pitch:{}, {}",
                            self.yaw_angle_f, self.yaw_angle, self.pitch_angle_f, self.pitch_angle
                        );

                        Msg::AngleSet
                    } else {
                        Msg::None
                    }
                }
                _ => Msg::None,
            };

            frame.release();
            return Ok(ret_msg);
        }
        Ok(Msg::None)
    }

    // /**
    //  * TODO: Implement angle upstream message!
    //  */
    // fn send_upstream_message(&mut self, angle: &[u8; 2]) -> Result<()> {
    //     if let Ok(mut wg) = self.sender.grant(2) {
    //         wg.to_commit(2);
    //         wg.copy_from_slice(angle);
    //         wg.commit(2);
    //     }
    //     Ok(())
    // }

    // fn send_upstream_message_u8(&mut self, msg: Msg, val: u8) -> Result<()> {
    //     info!("Sending info Msg:{:x}, Val:{:x}", msg as u8, val);

    //     if let Ok(mut wg) = self.sender.grant(2) {
    //         wg.to_commit(1);
    //         wg[0] = msg as u8;
    //         wg[1] = val;
    //         wg.commit(2);
    //     }
    //     Ok(())
    // }

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
}
