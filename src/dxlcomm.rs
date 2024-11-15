use anyhow::Result;
// use esp_idf_hal::io::Write;
use log::*;
// use num_traits::ToBytes;
use std::time::{
    // Duration,
    SystemTime,
};

use esp_idf_hal::{
    delay,
    gpio::*,
    task::*,
    uart::{
        AsyncUartDriver,
        //  Uart,
        UartDriver,
    },
};
// use esp_idf_svc::systime::EspSystemTime;
// use esp_idf_sys::EspError;

use bbqueue::framed::{FrameConsumer, FrameProducer};
use crc16::*;

use num_derive::FromPrimitive;
extern crate bincode;

use crate::Msg;

use crate::{
    ANGLE_REPORT_INTERVAL_MS, DYNAMIXEL_TASK_INTERVAL_MS, MSG_DXL_BUF_DOWNSTREAM,
    MSG_DXL_BUF_UPSTREAM,
};

pub const DYNAMIXEL_RESOLUTION: f32 = 4096.0;
pub const PITCH_LIMIT_ANGLE: f32 = 91.0;
pub const YAW_LIMIT_ANGLE: f32 = 181.0;

const DYNAMIXEL_PACKET_HEADER: [u8; 4] = [0xFFu8, 0xFFu8, 0xFDu8, 0x00u8];

const RESPONSE_BUF_SIZE: usize = 15;
const DYNAMIXEL_INITIAL_MAX_SPEED: u16 = 100;
const DYNAMIXEL_INITIAL_MAX_ACCELARATION: u16 = 3;
const DYNAMIXEL_INITIAL_POSITOIN_P_GAIN: u16 = 640;
const DYNAMIXEL_INITIAL_POSITOIN_I_GAIN: u16 = 0;
const DYNAMIXEL_INITIAL_POSITOIN_D_GAIN: u16 = 4000;

#[allow(dead_code)]
#[derive(FromPrimitive, Clone, Copy, PartialEq)]
enum DxlInstruction {
    Ping = 1,
    Read = 2,
    Write = 3,
}

enum DxlAddress {
    PositionPGain = 0x54, //84
    PositionIGain = 0x55, //85
    PositionDGain = 0x56, //86

    ProfileAccel = 0x6c,    //108,
    ProfileVelocity = 0x70, // 112
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
    max_speed: u16,
    max_accel: u16,
    position_p_gain: u16,
    position_i_gain: u16,
    position_d_gain: u16,
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
            max_speed: DYNAMIXEL_INITIAL_MAX_SPEED,
            max_accel: DYNAMIXEL_INITIAL_MAX_ACCELARATION,
            position_p_gain: DYNAMIXEL_INITIAL_POSITOIN_P_GAIN,
            position_i_gain: DYNAMIXEL_INITIAL_POSITOIN_I_GAIN,
            position_d_gain: DYNAMIXEL_INITIAL_POSITOIN_D_GAIN,
            angle_report_timestamp,
            response_buf,
        }
    }

    pub fn init(&mut self) -> Result<()> {
        self.ping(Motor::Yaw)?;
        self.ping(Motor::Pitch)?;
        delay::Ets::delay_ms(10);

        self.set_led(Motor::Yaw, true)?;
        self.set_led(Motor::Pitch, true)?;
        delay::Ets::delay_ms(10);

        self.set_max_speed(self.max_speed as u32)?;
        self.set_max_accel(self.max_accel as u32)?;
        delay::Ets::delay_ms(10);

        self.set_position_gains(
            self.position_p_gain as u32,
            self.position_i_gain as u32,
            self.position_d_gain as u32,
        )?;
        delay::Ets::delay_ms(10);

        self.enable_torque(Motor::Yaw, true)?;
        self.enable_torque(Motor::Pitch, true)?;
        delay::Ets::delay_ms(10);

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
                Msg::MaxSpeedSet => {
                    info!("Setting Max Speed:{}", self.max_speed);
                    self.set_max_speed(self.max_speed as u32)?;
                }
                Msg::MaxAccelSet => {
                    info!("Setting Max Acceleration:{}", self.max_accel);
                    self.set_max_accel(self.max_accel as u32)?;
                }
                Msg::PositionGainSet => {
                    info!(
                        "Setting Positon Gains P:{} I:{} D:{}",
                        self.position_p_gain, self.position_i_gain, self.position_d_gain
                    );
                    self.set_position_gains(
                        self.position_p_gain as u32,
                        self.position_d_gain as u32,
                        self.position_d_gain as u32,
                    )?;
                }
                _ => {}
            }
            // } else {
            //     None
        };
        if self.angle_report_timestamp.elapsed().unwrap().as_millis() > ANGLE_REPORT_INTERVAL_MS {
            self.angle_report_timestamp = SystemTime::now();
            let dynamixel_half_res = (DYNAMIXEL_RESOLUTION / 2.0) as i32;
            if let Ok(angle) = self.read_angle(Motor::Yaw) {
                if -dynamixel_half_res < angle && angle < dynamixel_half_res {
                    self.yaw_angle = angle;
                    self.yaw_angle_f = ((angle as f32 / DYNAMIXEL_RESOLUTION) * 360.0) - 180.0;
                    // info!("Yaw Angle:{}", self.pitch_angle_f);
                }
            }
            if let Ok(angle) = self.read_angle(Motor::Pitch) {
                if -dynamixel_half_res < angle && angle < dynamixel_half_res {
                    self.pitch_angle_f = ((angle as f32 / DYNAMIXEL_RESOLUTION) * 360.0) - 180.0;
                    // info!("Pitch Angle:{} {}", self.pitch_angle, self.yaw_angle_f);
                }
            }
            self.send_upstream_message_angles(
                Msg::AngleReport,
                self.yaw_angle as i16,
                self.pitch_angle as i16,
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

    fn set_max_speed(&mut self, max_speed: u32) -> Result<()> {
        self.set_control_table_value(Motor::Yaw, max_speed, DxlAddress::ProfileVelocity)?;
        delay::Ets::delay_ms(1);
        self.set_control_table_value(Motor::Pitch, max_speed, DxlAddress::ProfileVelocity)?;
        delay::Ets::delay_ms(1);
        // info!("set max speed done: {:x?}", self.response_buf);

        Ok(())
    }

    fn set_max_accel(&mut self, max_accel: u32) -> Result<()> {
        self.set_control_table_value(Motor::Yaw, max_accel, DxlAddress::ProfileAccel)?;
        delay::Ets::delay_ms(1);
        self.set_control_table_value(Motor::Pitch, max_accel, DxlAddress::ProfileAccel)?;
        delay::Ets::delay_ms(1);
        // info!("set max speed done: {:x?}", self.response_buf);

        Ok(())
    }

    fn set_position_gains(
        &mut self,
        position_p_gain: u32,
        position_i_gain: u32,
        position_d_gain: u32,
    ) -> Result<()> {
        // Yaw settings
        self.set_control_table_value(Motor::Yaw, position_p_gain, DxlAddress::PositionPGain)?;
        delay::Ets::delay_ms(1);
        self.set_control_table_value(Motor::Yaw, position_i_gain, DxlAddress::PositionIGain)?;
        delay::Ets::delay_ms(1);
        self.set_control_table_value(Motor::Yaw, position_d_gain, DxlAddress::PositionDGain)?;
        delay::Ets::delay_ms(1);

        // Pitch settings
        self.set_control_table_value(Motor::Pitch, position_p_gain, DxlAddress::PositionPGain)?;
        delay::Ets::delay_ms(1);
        self.set_control_table_value(Motor::Pitch, position_i_gain, DxlAddress::PositionIGain)?;
        delay::Ets::delay_ms(1);
        self.set_control_table_value(Motor::Pitch, position_d_gain, DxlAddress::PositionDGain)?;
        delay::Ets::delay_ms(1);
        info!(
            "set position gain done: P:{} I:{} D:{}",
            self.position_p_gain, self.position_i_gain, self.position_d_gain
        );

        Ok(())
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

        info!("Torque Msg: {:02X?}", msg_buf);

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

    fn set_control_table_value(
        &mut self,
        id: Motor,
        setting_value: u32,
        address: DxlAddress,
    ) -> Result<()> {
        let mut msg_buf = DYNAMIXEL_PACKET_HEADER.to_vec();
        msg_buf.push(id as u8);
        // Length
        msg_buf.push(0x09u8);
        msg_buf.push(0x0u8);
        // Instruction: Write
        msg_buf.push(DxlInstruction::Write as u8);
        // Address
        msg_buf.push(address as u8);
        msg_buf.push(0x0u8);
        // speed setting
        msg_buf.extend_from_slice(&setting_value.to_le_bytes());

        // Get CRC
        let mut crc_state = State::<BUYPASS>::new();
        crc_state.update(&msg_buf);
        let crc_res = crc_state.get();
        msg_buf.extend_from_slice(&crc_res.to_le_bytes());

        info!(
            "Set control table value Msg: {} {:02X?}",
            setting_value, msg_buf
        );
        self.read_write_dxl_message(&msg_buf, 11)?;

        Ok(())
    }

    // Handle message from OSC downstream
    fn check_downstream_message(&mut self) -> Result<Msg> {
        if let Some(frame) = self.receiver.read() {
            // info!(
            //     "Dynamixel downstream msg:{:x} Size:{}",
            //     frame[0],
            //     frame.len()
            // );
            let msg_type: Option<Msg> = num::FromPrimitive::from_u8(frame[0]);

            let ret_msg = match msg_type {
                Some(Msg::AngleSet) => {
                    if frame.len() == 5 {
                        self.yaw_angle = i16::from_be_bytes([frame[1], frame[2]]) as i32;
                        self.pitch_angle = i16::from_be_bytes([frame[3], frame[4]]) as i32;
                        // info!(
                        //     "Yaw:{}, {} Pitch:{}, {}",
                        //     self.yaw_angle_f, self.yaw_angle, self.pitch_angle_f, self.pitch_angle
                        // );

                        Msg::AngleSet
                    } else {
                        Msg::None
                    }
                }
                Some(Msg::MaxSpeedSet) => {
                    if frame.len() == 3 {
                        self.max_speed = u16::from_be_bytes([frame[1], frame[2]]);

                        Msg::MaxSpeedSet
                    } else {
                        Msg::None
                    }
                }
                Some(Msg::MaxAccelSet) => {
                    if frame.len() == 3 {
                        self.max_accel = u16::from_be_bytes([frame[1], frame[2]]);

                        Msg::MaxAccelSet
                    } else {
                        Msg::None
                    }
                }
                Some(Msg::PositionGainSet) => {
                    if frame.len() == 7 {
                        self.position_p_gain = u16::from_be_bytes([frame[1], frame[2]]);
                        self.position_i_gain = u16::from_be_bytes([frame[3], frame[4]]);
                        self.position_d_gain = u16::from_be_bytes([frame[5], frame[6]]);

                        Msg::PositionGainSet
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

    fn send_upstream_message_angles(&mut self, msg: Msg, yaw: i16, pitch: i16) -> Result<()> {
        if let Ok(mut wg) = self.sender.grant(5) {
            wg.to_commit(1);
            wg[0] = msg as u8;
            wg[1..3].copy_from_slice(&(yaw.to_be_bytes()));
            wg[3..5].copy_from_slice(&(pitch.to_be_bytes()));
            wg.commit(5);
        }

        Ok(())
    }

    #[allow(dead_code)]
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
