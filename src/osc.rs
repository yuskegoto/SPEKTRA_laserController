use anyhow::Result;
use log::*;
use num_derive::FromPrimitive;

use esp_idf_hal::reset::restart;
use num_traits::ToBytes;

extern crate num;
extern crate num_derive;

use std::net::{Ipv4Addr, SocketAddrV4, UdpSocket};
use std::time::Duration;

use bbqueue::framed::{FrameConsumer, FrameProducer};
use rosc::{self, OscMessage, OscPacket, OscType};
extern crate bincode;

const OSC_LISTEN_INTERVAL_MS: Duration = Duration::from_millis(1);

use crate::{MSG_BUF_DOWNSTREAM, MSG_BUF_UPSTREAM, MSG_DXL_BUF_DOWNSTREAM, MSG_DXL_BUF_UPSTREAM};

#[allow(dead_code)]
#[derive(FromPrimitive, Clone, Copy, PartialEq)]
pub enum Msg {
    None = 0,
    Error = 0xFF,

    AngleSet = 0x01,
    AngleReport = 0x02,
    MaxSpeedSet = 0x03,
    MaxAccelSet = 0x04,

    ColorSet = 0x10,
    MaxDutySet = 0x12,

    // Controller messages
    Boot = 0x70,
    Version = 0x71,
    SetDestIp = 0x72,
}

pub struct OscReceiver {
    sock: UdpSocket,
    buf: [u8; rosc::decoder::MTU],
    sender: FrameProducer<'static, MSG_BUF_DOWNSTREAM>,
    dxl_sender: FrameProducer<'static, MSG_DXL_BUF_DOWNSTREAM>,
}

impl OscReceiver {
    pub fn new(
        ip: embedded_svc::ipv4::Ipv4Addr,
        recv_port: u16,
        sender: FrameProducer<'static, MSG_BUF_DOWNSTREAM>,
        dxl_sender: FrameProducer<'static, MSG_DXL_BUF_DOWNSTREAM>,
    ) -> Self {
        let recv_addr = SocketAddrV4::new(ip, recv_port);
        let sock = UdpSocket::bind(recv_addr).unwrap();
        let buf = [0u8; rosc::decoder::MTU];

        info!("Listening to {recv_addr}");

        Self {
            sock,
            buf,
            sender,
            dxl_sender,
        }
    }

    pub fn run(&mut self) -> Result<()> {
        match self.sock.recv_from(&mut self.buf) {
            Ok((size, _addr)) => {
                // info!("Received packet with size {size} from: {_addr}");

                let res = rosc::decoder::decode_udp(&self.buf[..size]);
                match res {
                    Ok((_, packet)) => {
                        match packet {
                            OscPacket::Message(msg) => {
                                // info!("OSC address: {}", msg.addr);
                                // info!("OSC arguments: {:?}, len:{}", msg.args, msg.args.len());

                                match msg.addr.as_str() {
                                    "/reset" => {
                                        // Reset!
                                        self.reset_sequence();
                                    }
                                    "/version" => {
                                        self.send_downstream_buffer(Msg::Version, &[0u8]);
                                    }

                                    "/color" => {
                                        if msg.args.len() == 3 {
                                            let mut commandbuf = vec![];
                                            for arg in msg.args.iter() {
                                                let val = arg.clone().float();
                                                if let Some(v) = val {
                                                    let serialized =
                                                        bincode::serialize(&v).unwrap();
                                                    commandbuf.extend_from_slice(&serialized);
                                                }
                                            }
                                            // info!("Color set:{:02X?}", commandbuf);
                                            self.send_downstream_buffer(Msg::ColorSet, &commandbuf);
                                        }
                                    }
                                    "/setangle" => {
                                        if msg.args.len() == 2 {
                                            let mut commandbuf = vec![];
                                            for arg in msg.args.iter() {
                                                let val = arg.clone().float();
                                                if let Some(v) = val {
                                                    let serialized =
                                                        bincode::serialize(&v).unwrap();
                                                    commandbuf.extend_from_slice(&serialized);
                                                }
                                            }
                                            // info!("Angle set:{:02X?}", commandbuf);
                                            self.send_dxl_downstream_buffer(
                                                Msg::AngleSet,
                                                &commandbuf,
                                            );
                                        }
                                    }

                                    "/setmaxduty" => {
                                        if msg.args.len() == 1 {
                                            let mut commandbuf = vec![];
                                            for arg in msg.args.iter() {
                                                let val = arg.clone().float();
                                                if let Some(v) = val {
                                                    let serialized =
                                                        bincode::serialize(&v).unwrap();
                                                    commandbuf.extend_from_slice(&serialized);
                                                }
                                            }
                                            self.send_downstream_buffer(
                                                Msg::MaxDutySet,
                                                &commandbuf,
                                            );
                                        }
                                    }

                                    "/setmaxspeed" => {
                                        if msg.args.len() == 1 {
                                            let mut commandbuf = vec![];
                                            for arg in msg.args.iter() {
                                                let val = arg.clone().int();
                                                if let Some(v) = val {
                                                    let settings =
                                                        arg.clone().int().unwrap() as u16;
                                                    commandbuf.extend_from_slice(
                                                        &(settings.to_be_bytes()),
                                                    );
                                                }
                                            }
                                            self.send_dxl_downstream_buffer(
                                                Msg::MaxSpeedSet,
                                                &commandbuf,
                                            );
                                        }
                                    }

                                    "/setmaxaccel" => {
                                        if msg.args.len() == 1 {
                                            let mut commandbuf = vec![];
                                            for arg in msg.args.iter() {
                                                let val = arg.clone().int();
                                                if let Some(v) = val {
                                                    let settings =
                                                        arg.clone().int().unwrap() as u16;
                                                    commandbuf.extend_from_slice(
                                                        &(settings.to_be_bytes()),
                                                    );
                                                }
                                            }
                                            self.send_dxl_downstream_buffer(
                                                Msg::MaxAccelSet,
                                                &commandbuf,
                                            );
                                        }
                                    }

                                    "/setdestip" => {
                                        if msg.args.len() == 4 {
                                            let mut commandbuf = vec![];
                                            for arg in msg.args.iter() {
                                                let ip = arg.clone().int().unwrap();
                                                commandbuf.push((ip & 0xFF) as u8);
                                            }
                                            self.send_downstream_buffer(
                                                Msg::SetDestIp,
                                                &commandbuf,
                                            );
                                        }
                                    }

                                    _ => {}
                                }
                            }
                            OscPacket::Bundle(bundle) => {
                                info!("OSC Bundle: {bundle:?}");
                            }
                        }
                    }
                    Err(e) => {
                        error!("Error receiving OSC msg: {e}");
                        ()
                    }
                }
                ()
            }
            Err(e) => {
                error!("Error receiving from socket: {e}");
            }
        }
        Ok(())
    }

    /**
     * Send message to device management thread
     */
    fn send_downstream_buffer(&mut self, header: Msg, content: &[u8]) {
        let mut msg_buf = vec![];
        msg_buf.push(header as u8);
        for ct in content.iter() {
            msg_buf.push(*ct);
        }
        // info!("Downstream buf:{:02X?}", msg_buf);

        let sz = msg_buf.len();
        if let Ok(mut wg) = self.sender.grant(sz) {
            wg.to_commit(sz);
            wg.copy_from_slice(msg_buf.as_slice());
            wg.commit(sz);
        } else {
            error!("Downstream buffer overflow!");
        }
    }

    /**
     * Send message to dynamixel communication task
     */
    fn send_dxl_downstream_buffer(&mut self, header: Msg, content: &[u8]) {
        let mut msg_buf = vec![];
        msg_buf.push(header as u8);
        for ct in content.iter() {
            msg_buf.push(*ct);
        }
        // info!("Dxl downstream buf:{:02X?}", msg_buf);

        let sz = msg_buf.len();
        if let Ok(mut wg) = self.dxl_sender.grant(sz) {
            wg.to_commit(sz);
            wg.copy_from_slice(msg_buf.as_slice());
            wg.commit(sz);
        } else {
            error!("Dxl downstream buffer overflow!");
        }
    }

    /**
     * Reset the device on /reset 0 command!
     */
    fn reset_sequence(&self) {
        // Wait a little bit until all buffer is cleared etc
        std::thread::sleep(Duration::from_millis(100));
        restart();
    }

    /**
     * Sleep until next interval
     */
    pub fn idle(&self) {
        std::thread::sleep(OSC_LISTEN_INTERVAL_MS);
    }
}

///////////////////////////////////////////////////////
// Upstream Messenger
// Upstream Message Buffer -> OSC Send out
pub struct OscSender {
    sock: UdpSocket,
    consumer: FrameConsumer<'static, MSG_BUF_UPSTREAM>,
    dxl_consumer: FrameConsumer<'static, MSG_DXL_BUF_UPSTREAM>,
    dest_addr: SocketAddrV4,
    device_no: u8,
}

impl OscSender {
    pub fn new(
        dest_ip: embedded_svc::ipv4::Ipv4Addr,
        dest_port: u16,
        host_ip: embedded_svc::ipv4::Ipv4Addr,
        host_port: u16,
        consumer: FrameConsumer<'static, MSG_BUF_UPSTREAM>,
        dxl_consumer: FrameConsumer<'static, MSG_DXL_BUF_UPSTREAM>,
        device_no: u8,
    ) -> Self {
        let dest_addr = SocketAddrV4::new(dest_ip, dest_port);
        let host_addr = SocketAddrV4::new(host_ip, host_port);
        let sock = UdpSocket::bind(host_addr).unwrap();

        Self {
            sock,
            consumer,
            dxl_consumer,
            dest_addr,
            device_no,
        }
    }

    /**
     * Check machine status and if available, dispatches OSC message to upstream
     */
    pub fn run(&mut self) -> Result<()> {
        self.handle_device_msg()?;
        self.handle_dynamixel_msg()?;
        // // Check if angle data is coming
        // if let Some(frame) = self.consumer.read() {
        //     if frame.len() < 2 {
        //         frame.release();
        //         return Ok(());
        //     }

        //     let mut buf = vec![];
        //     let msg_type: Option<Msg> = num::FromPrimitive::from_u8(frame[0]);
        //     let mut addr_str = match msg_type {
        //         Some(Msg::AngleReport) => {
        //             if frame.len() == 9 {
        //                 let yawangle: f32 = bincode::deserialize(&frame[1..5]).unwrap();
        //                 buf.push(OscType::Float(yawangle));
        //                 let pitchangle: f32 = bincode::deserialize(&frame[5..9]).unwrap();
        //                 buf.push(OscType::Float(pitchangle));
        //                 info!("Reporting Angle:{yawangle}, {pitchangle}");
        //                 "/angle".to_string()
        //             } else {
        //                 "/unknown".to_string()
        //             }
        //         }

        //         Some(Msg::SetDestIp) => {
        //             if frame.len() == 5 {
        //                 self.set_dest_ip(&frame[1..5]);
        //                 let ip_addr = self.dest_addr.ip().octets();
        //                 for ip in ip_addr.iter() {
        //                     buf.push(OscType::Int(*ip as i32));
        //                 }

        //                 "/destip".to_string()
        //             } else {
        //                 "/unknown".to_string()
        //             }
        //         }

        //         _ => {
        //             // Append header to the packet for debug
        //             buf.push(OscType::Int(frame[0] as i32));
        //             buf.push(OscType::Int(frame[1] as i32));
        //             "/unknown".to_string()
        //         }
        //     };

        //     frame.release();
        //     addr_str += self.device_no.to_string().as_str();

        //     let msg_buf = rosc::encoder::encode(&OscPacket::Message(OscMessage {
        //         addr: addr_str,
        //         args: buf,
        //     }))?;

        //     let ret = self.sock.send_to(&msg_buf, self.dest_addr);
        //     match ret {
        //         Ok(_) => {
        //             // info!("Sent out osc msg to PC");
        //         }
        //         Err(e) => {
        //             error!("Error sending out osc msg to PC: {e}");
        //         }
        //     }
        // };

        Ok(())
    }

    /**
     * Sleep until next interval
     */
    pub fn idle(&self) {
        std::thread::sleep(OSC_LISTEN_INTERVAL_MS);
    }

    fn handle_device_msg(&mut self) -> Result<()> {
        // Check if message from device task is coming
        if let Some(frame) = self.consumer.read() {
            if frame.len() < 2 {
                frame.release();
                return Ok(());
            }

            let mut buf = vec![];
            let msg_type: Option<Msg> = num::FromPrimitive::from_u8(frame[0]);
            let mut addr_str = match msg_type {
                // Some(Msg::AngleReport) => {
                //     if frame.len() == 9 {
                //         let yawangle: f32 = bincode::deserialize(&frame[1..5]).unwrap();
                //         buf.push(OscType::Float(yawangle));
                //         let pitchangle: f32 = bincode::deserialize(&frame[5..9]).unwrap();
                //         buf.push(OscType::Float(pitchangle));
                //         info!("Reporting Angle:{yawangle}, {pitchangle}");
                //         "/angle".to_string()
                //     } else {
                //         "/unknown".to_string()
                //     }
                // }
                Some(Msg::SetDestIp) => {
                    if frame.len() == 5 {
                        self.set_dest_ip(&frame[1..5]);
                        let ip_addr = self.dest_addr.ip().octets();
                        for ip in ip_addr.iter() {
                            buf.push(OscType::Int(*ip as i32));
                        }

                        "/destip".to_string()
                    } else {
                        "/unknown".to_string()
                    }
                }

                _ => {
                    // Append header to the packet for debug
                    buf.push(OscType::Int(frame[0] as i32));
                    buf.push(OscType::Int(frame[1] as i32));
                    "/unknown".to_string()
                }
            };

            frame.release();
            addr_str += self.device_no.to_string().as_str();

            let msg_buf = rosc::encoder::encode(&OscPacket::Message(OscMessage {
                addr: addr_str,
                args: buf,
            }))?;

            let ret = self.sock.send_to(&msg_buf, self.dest_addr);
            match ret {
                Ok(_) => {
                    // info!("Sent out osc msg to PC");
                }
                Err(e) => {
                    error!("Error sending out osc msg to PC: {e}");
                }
            }
        };

        Ok(())
    }

    fn handle_dynamixel_msg(&mut self) -> Result<()> {
        // Check if message from the Dynamixel task is coming
        if let Some(frame) = self.dxl_consumer.read() {
            if frame.len() < 2 {
                frame.release();
                return Ok(());
            }

            let mut buf = vec![];
            let msg_type: Option<Msg> = num::FromPrimitive::from_u8(frame[0]);
            let mut addr_str = match msg_type {
                Some(Msg::AngleReport) => {
                    if frame.len() == 9 {
                        let yawangle: f32 = bincode::deserialize(&frame[1..5]).unwrap();
                        buf.push(OscType::Float(yawangle));
                        let pitchangle: f32 = bincode::deserialize(&frame[5..9]).unwrap();
                        buf.push(OscType::Float(pitchangle));
                        // info!("Reporting Angle:{yawangle}, {pitchangle}");
                        "/angle".to_string()
                    } else {
                        "/unknown".to_string()
                    }
                }

                // Some(Msg::SetDestIp) => {
                //     if frame.len() == 5 {
                //         self.set_dest_ip(&frame[1..5]);
                //         let ip_addr = self.dest_addr.ip().octets();
                //         for ip in ip_addr.iter() {
                //             buf.push(OscType::Int(*ip as i32));
                //         }

                //         "/destip".to_string()
                //     } else {
                //         "/unknown".to_string()
                //     }
                // }
                _ => {
                    // Append header to the packet for debug
                    buf.push(OscType::Int(frame[0] as i32));
                    buf.push(OscType::Int(frame[1] as i32));
                    "/unknown".to_string()
                }
            };

            frame.release();
            addr_str += self.device_no.to_string().as_str();

            let msg_buf = rosc::encoder::encode(&OscPacket::Message(OscMessage {
                addr: addr_str,
                args: buf,
            }))?;

            let ret = self.sock.send_to(&msg_buf, self.dest_addr);
            match ret {
                Ok(_) => {
                    // info!("Sent out osc msg to PC");
                }
                Err(e) => {
                    error!("Error sending out osc msg to PC: {e}");
                }
            }
        };

        Ok(())
    }

    /**
     *  Send boot msg to the PC
     */
    pub fn send_bootmsg(&self) -> Result<()> {
        let msg_buf = rosc::encoder::encode(&OscPacket::Message(OscMessage {
            addr: "/boot".to_string() + self.device_no.to_string().as_str(),
            args: vec![OscType::Int(self.device_no as i32)],
        }))?;

        if let Err(e) = self.sock.send_to(&msg_buf, self.dest_addr) {
            error!("Error sending OSC{e}");
        }

        Ok(())
    }

    fn set_dest_ip(&mut self, ip: &[u8]) {
        if ip.len() == 4 {
            let mut newip = [0u8; 4];
            newip.copy_from_slice(&ip);
            let dest_ip = Ipv4Addr::from(newip);
            self.dest_addr.set_ip(dest_ip);
            info!("New Dest IP:{:?}", dest_ip);
        }
    }
}
