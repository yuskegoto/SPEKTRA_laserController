use anyhow::Result;
use esp_idf_hal::io::Write;
use log::*;
use num_traits::ToBytes;
use std::time::Duration;

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
    MODBUS_LISTEN_INTERVAL_MS, MSG_DXL_BUF_DOWNSTREAM, MSG_DXL_BUF_UPSTREAM, SOFTWARE_VERSION,
};

const DYNAMIXEL_RESOLUTION: f32 = 4096.0;
const PITCH_LIMIT_ANGLE: f32 = 91.0;
const YAW_LIMIT_ANGLE: f32 = 181.0;

#[allow(dead_code)]
#[derive(FromPrimitive, Clone, Copy, PartialEq)]
pub enum DynamixelMsg {
    Angle = 0,
    Reset = 1,
    Brake = 2,
    BrakeState = 3,
}

enum Motor {
    Pitch = 1,
    Yaw = 2,
}

pub struct Dynamixel<'a, V: Pin> {
    sender: FrameProducer<'static, MSG_DXL_BUF_UPSTREAM>,
    receiver: FrameConsumer<'static, MSG_DXL_BUF_DOWNSTREAM>,
    // dxl: UartDriver<'a>,
    dxl: AsyncUartDriver<'a, UartDriver<'a>>,
    dxl_en: PinDriver<'a, V, Output>,
    pitch_angle: i32,
    yaw_angle: i32,
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
        Self {
            sender,
            receiver,
            dxl,
            dxl_en,
            pitch_angle: 0,
            yaw_angle: 0,
        }
    }

    pub fn init(&mut self) -> Result<()> {
        // self.ping(1u8)?;
        self.read_angle(1u8)?;

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
                Msg::AngleSet => {}
                _ => {}
            }
            // } else {
            //     None
        };

        // If msg is available, send via Dynamixel
        Ok(())
    }

    /**
     * Sleep until next interval
     */
    pub fn idle(&self) {
        std::thread::sleep(MODBUS_LISTEN_INTERVAL_MS);
    }

    // /**
    //  * Send alarm reset message to AZ motor
    //  */
    // fn alarm_reset(&mut self, motor: Motor) -> Result<()> {
    //     let m = motor as u8;
    //     let mut msg_buf = vec![m];
    //     msg_buf.push(MODBUS_MSG_FUNCTION_CODE_WRITE);
    //     msg_buf.extend_from_slice(&(MODBUS_MSG_DRIVER_INPUT_REG_ADDR.to_be_bytes()));

    //     let register_state = MODBUS_MSG_DRIVER_INPUT_REG_ALM_RST | MODBUS_MSG_DRIVER_INPUT_REG_C_ON;
    //     msg_buf.extend_from_slice(&(register_state.to_be_bytes()));

    //     // Get CRC
    //     let mut crc_state = State::<MODBUS>::new();
    //     crc_state.update(&msg_buf);
    //     let crc_res = crc_state.get();
    //     msg_buf.extend_from_slice(&crc_res.to_le_bytes());

    //     info!("Alarm Reset Msg: {:x?}", msg_buf);

    //     let mut response_buf: [u8; 8] = [0u8; 8];
    //     self.write_dxl_message(&msg_buf, &mut response_buf)?;

    //     msg_buf.clear();
    //     msg_buf.push(m);
    //     msg_buf.push(MODBUS_MSG_FUNCTION_CODE_WRITE);
    //     msg_buf.extend_from_slice(&(MODBUS_MSG_DRIVER_INPUT_REG_ADDR.to_be_bytes()));

    //     // Clear the register
    //     let register_state = MODBUS_MSG_DRIVER_INPUT_REG_C_ON;
    //     msg_buf.extend_from_slice(&(register_state.to_be_bytes()));
    //     // Get CRC
    //     let mut crc_state = State::<MODBUS>::new();
    //     crc_state.update(&msg_buf);
    //     let crc_res = crc_state.get();
    //     msg_buf.extend_from_slice(&crc_res.to_le_bytes());

    //     info!("Alarm Release Msg: {:x?}", msg_buf);

    //     self.write_dxl_message(&msg_buf, &mut response_buf)?;

    //     Ok(())
    // }

    // /**
    //  * Send ZHome message to AZ motor
    //  */
    // fn zhome(&mut self, motor: Motor) -> Result<()> {
    //     let m = motor as u8;
    //     let mut msg_buf = vec![m];
    //     msg_buf.push(MODBUS_MSG_FUNCTION_CODE_WRITE);
    //     msg_buf.extend_from_slice(&(MODBUS_MSG_DRIVER_INPUT_REG_ADDR.to_be_bytes()));

    //     let register_state = MODBUS_MSG_DRIVER_INPUT_REG_ZHOME | MODBUS_MSG_DRIVER_INPUT_REG_C_ON;
    //     msg_buf.extend_from_slice(&(register_state.to_be_bytes()));

    //     // Get CRC
    //     let mut crc_state = State::<MODBUS>::new();
    //     crc_state.update(&msg_buf);
    //     let crc_res = crc_state.get();
    //     msg_buf.extend_from_slice(&crc_res.to_le_bytes());

    //     info!("ZHome Msg: {:x?}", msg_buf);

    //     let mut response_buf: [u8; 8] = [0u8; 8];
    //     self.write_dxl_message(&msg_buf, &mut response_buf)?;

    //     msg_buf.clear();
    //     msg_buf.push(m);
    //     msg_buf.push(MODBUS_MSG_FUNCTION_CODE_WRITE);
    //     msg_buf.extend_from_slice(&(MODBUS_MSG_DRIVER_INPUT_REG_ADDR.to_be_bytes()));

    //     // Clear the register
    //     let register_state = MODBUS_MSG_DRIVER_INPUT_REG_C_ON;
    //     msg_buf.extend_from_slice(&(register_state.to_be_bytes()));
    //     // Get CRC
    //     let mut crc_state = State::<MODBUS>::new();
    //     crc_state.update(&msg_buf);
    //     let crc_res = crc_state.get();
    //     msg_buf.extend_from_slice(&crc_res.to_le_bytes());

    //     info!("Zhome Msg: {:x?}", msg_buf);

    //     self.write_dxl_message(&msg_buf, &mut response_buf)?;

    //     Ok(())
    // }

    // /**
    //  * Send brake message to AZ motor
    //  */
    // fn brake(&mut self, motor: Motor, hold: bool) -> Result<()> {
    //     info!("Braking Hold:{}", hold);

    //     let m = motor as u8;
    //     let mut msg_buf = vec![m];
    //     msg_buf.push(MODBUS_MSG_FUNCTION_CODE_WRITE);
    //     msg_buf.extend_from_slice(&(MODBUS_MSG_DRIVER_INPUT_REG_ADDR.to_be_bytes()));

    //     let register_state = if hold {
    //         MODBUS_MSG_DRIVER_INPUT_REG_C_ON
    //     } else {
    //         0
    //     };
    //     msg_buf.extend_from_slice(&(register_state.to_be_bytes()));

    //     // Get CRC
    //     let mut crc_state = State::<MODBUS>::new();
    //     crc_state.update(&msg_buf);
    //     let crc_res = crc_state.get();
    //     msg_buf.extend_from_slice(&crc_res.to_le_bytes());

    //     info!("Brake Msg: {:x?}", msg_buf);

    //     let mut response_buf: [u8; 8] = [0u8; 8];
    //     self.write_dxl_message(&msg_buf, &mut response_buf)?;

    //     msg_buf.clear();
    //     msg_buf.push(m);
    //     msg_buf.push(MODBUS_MSG_FUNCTION_CODE_WRITE);
    //     msg_buf.extend_from_slice(&(MODBUS_MSG_DRIVER_INPUT_REG_ADDR.to_be_bytes()));

    //     let register_state = if hold {
    //         0
    //     } else {
    //         MODBUS_MSG_DRIVER_INPUT_REG_C_ON
    //     };
    //     msg_buf.extend_from_slice(&(register_state.to_be_bytes()));

    //     // Get CRC
    //     let mut crc_state = State::<MODBUS>::new();
    //     crc_state.update(&msg_buf);
    //     let crc_res = crc_state.get();
    //     msg_buf.extend_from_slice(&crc_res.to_le_bytes());

    //     info!("Release Msg: {:x?}", msg_buf);

    //     self.write_dxl_message(&msg_buf, &mut response_buf)?;

    //     Ok(())
    // }

    // /**
    //  * Send direct dirve packet to motor
    //  */
    // fn set_angle(
    //     &mut self,
    //     motor: Motor,
    //     angle: i32,
    //     speed: u32,
    //     accel: u32,
    //     decel: u32,
    //     current: u32,
    // ) -> Result<()> {
    //     let m = motor as u8;
    //     let mut msg_buf = vec![m];
    //     msg_buf.push(MODBUS_MSG_FUNCTION_CODE_SEQUENTIAL_WRITE);
    //     msg_buf.extend_from_slice(&(MODBUS_MSG_DIRECT_DRIVE_ADDR.to_be_bytes()));

    //     let write_register_count = 16u16;
    //     msg_buf.extend_from_slice(&(write_register_count.to_be_bytes()));
    //     msg_buf.push(write_register_count as u8 * 2);
    //     // Action no. fixed
    //     msg_buf.extend_from_slice(&(0u32).to_be_bytes());
    //     // absolute positioning
    //     msg_buf.extend_from_slice(&(1u32).to_be_bytes());

    //     // position
    //     msg_buf.extend_from_slice(&(angle).to_be_bytes());

    //     // speed
    //     msg_buf.extend_from_slice(&(speed).to_be_bytes());

    //     // accel
    //     msg_buf.extend_from_slice(&(accel).to_be_bytes());

    //     // decel
    //     msg_buf.extend_from_slice(&(decel).to_be_bytes());

    //     // current
    //     msg_buf.extend_from_slice(&(current).to_be_bytes());

    //     // write flag
    //     msg_buf.extend_from_slice(&(1u32).to_be_bytes());

    //     // Get CRC
    //     let mut crc_state = State::<MODBUS>::new();
    //     crc_state.update(&msg_buf);
    //     let crc_res = crc_state.get();
    //     msg_buf.extend_from_slice(&crc_res.to_le_bytes());

    //     // info!("Angle Msg: {:x?}", msg_buf);

    //     let mut response_buf: [u8; 8] = [0u8; 8];
    //     self.write_dxl_message(&msg_buf, &mut response_buf)?;

    //     Ok(())
    // }

    fn write_dxl_message(&mut self, msg_buf: &[u8], response_buf: &mut [u8; 8]) -> Result<()> {
        // Write message
        let mut res_buf: [u8; 14] = [0u8; 14];

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
                            // block_on(async {
                            let res = self.dxl.read(&mut res_buf).await;
                            match res {
                                Ok(_) => {
                                    info!("Dynamixel read success: {:x?}", res_buf);
                                }
                                Err(e) => {
                                    error!("Dynamixel read failed: {:?}", e);
                                }
                            }
                            // });
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

        // std::thread::sleep(Duration::from_millis(1));
        Ok(())
    }

    fn read_dxl_message(&mut self, msg_buf: &[u8], response_buf: &mut [u8; 15]) -> Result<()> {
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
                            // block_on(async {
                            let res = self.dxl.read(response_buf).await;
                            match res {
                                Ok(_) => {
                                    // info!("Dynamixel read success: {:x?}", response_buf);
                                }
                                Err(e) => {
                                    error!("Dynamixel read failed: {:?}", e);
                                }
                            }
                            // });
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

        // std::thread::sleep(Duration::from_millis(1));
        Ok(())
    }

    fn ping(&mut self, id: u8) -> Result<()> {
        let mut msg_buf = vec![0xFFu8, 0xFFu8, 0xFDu8, 0x00u8];
        // ID
        msg_buf.push(id);
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

        let mut response_buf: [u8; 8] = [0u8; 8];
        self.write_dxl_message(&msg_buf, &mut response_buf)?;

        Ok(())
    }

    fn read_angle(&mut self, id: u8) -> Result<()> {
        let mut msg_buf = vec![0xFFu8, 0xFFu8, 0xFDu8, 0x00u8];
        // ID
        msg_buf.push(id);
        // Length
        msg_buf.push(0x07u8);
        msg_buf.push(0x00u8);
        // Instruction: Read
        msg_buf.push(0x02u8);

        msg_buf.push(0x84u8);
        msg_buf.push(0x00u8);
        msg_buf.push(0x04u8);
        msg_buf.push(0x00u8);

        // Get CRC
        // Should be fixed 0x15 0x1D
        let mut crc_state = State::<BUYPASS>::new();
        crc_state.update(&msg_buf);
        let crc_res = crc_state.get();
        info!("CRC: {:x?}", crc_res);
        msg_buf.extend_from_slice(&crc_res.to_le_bytes());

        info!("Position Msg: {:02X?}", msg_buf);

        let mut response_buf: [u8; 15] = [0u8; 15];
        self.read_dxl_message(&msg_buf, &mut response_buf)?;
        info!("Dynamixel answer: {:x?}", response_buf);

        let position = i32::from_le_bytes(response_buf[9..13].try_into().unwrap());
        info!("Position:{}", position);

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
                        let mut pitch: f32 = bincode::deserialize(&frame[1..5]).unwrap();
                        pitch = pitch.clamp(-PITCH_LIMIT_ANGLE, PITCH_LIMIT_ANGLE);
                        self.pitch_angle = (pitch / 360.0 * DYNAMIXEL_RESOLUTION) as i32;
                        let mut yaw: f32 = bincode::deserialize(&frame[5..9]).unwrap();
                        yaw = yaw.clamp(-YAW_LIMIT_ANGLE, YAW_LIMIT_ANGLE);
                        self.yaw_angle = (yaw / 360.0 * DYNAMIXEL_RESOLUTION) as i32;

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

    /**
     * TODO: Implement angle upstream message!
     */
    fn send_upstream_message(&mut self, angle: &[u8; 2]) -> Result<()> {
        if let Ok(mut wg) = self.sender.grant(2) {
            wg.to_commit(2);
            wg.copy_from_slice(angle);
            wg.commit(2);
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
