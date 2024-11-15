use anyhow::*;
// use esp_idf_hal::ledc::SpeedMode;
use log::*;
use std::result::Result::Ok;
// use std::sync::mpsc::channel;
// use std::sync::Arc;

use esp_idf_sys::{self as _};
use std::time::Duration; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use embedded_svc::ipv4::{self, IpInfo};
use embedded_svc::ipv4::{ClientConfiguration, ClientSettings, Mask, Subnet};
use esp_idf_svc::{eth::*, ping};
use esp_idf_svc::{eventloop::EspSystemEventLoop, netif, nvs::EspDefaultNvsPartition};

use esp_idf_hal::{
    delay,
    gpio,
    gpio::*,
    // i2c::*,
    ledc::{config as ledc_config, LedcDriver, LedcTimerDriver},
    peripherals::Peripherals,
    prelude::*,
    // task::*,
    uart::{
        config as uart_config,
        AsyncUartDriver,
        //  UartDriver
    },
};
// use esp_idf_svc::hal::prelude::*;

use std::net::Ipv4Addr;
use std::str::FromStr;

use bbqueue::BBBuffer;

use ws2812_esp32_rmt_driver::Ws2812Esp32Rmt;

mod osc;
use osc::*;

mod device;
use device::*;

mod dxlcomm;
use dxlcomm::*;

const SOFTWARE_VERSION: u8 = 1;

// OSC message queue buffer
const MSG_BUF_DOWNSTREAM: usize = 64;
const MSG_BUF_UPSTREAM: usize = 64;

// Dynamixel message queue buffer
const MSG_DXL_BUF_DOWNSTREAM: usize = 128; //64;
const MSG_DXL_BUF_UPSTREAM: usize = 64;

const LED_BRIGHTNESS: u8 = 10;
const LASER_BRIGHTNESS: f32 = 0.2;

static QUEUE_DOWNSTREAM: BBBuffer<MSG_BUF_DOWNSTREAM> = BBBuffer::new();
static QUEUE_UPSTREM: BBBuffer<MSG_BUF_UPSTREAM> = BBBuffer::new();

static QUEUE_DXL_DOWNSTREAM: BBBuffer<MSG_DXL_BUF_DOWNSTREAM> = BBBuffer::new();
static QUEUE_DXL_UPSTREAM: BBBuffer<MSG_DXL_BUF_UPSTREAM> = BBBuffer::new();

const GPIO_SLEEP_DURATION: Duration = Duration::from_millis(10);
const DYNAMIXEL_TASK_INTERVAL_MS: Duration = Duration::from_millis(10);

const LOCAL_OSC_RECV_PORT: u16 = 8000;
const LOCAL_OSC_SEND_PORT: u16 = 8001;
const REMOTE_OSC_PORT: u16 = 8100;

const DXL_BAUDRATE: u32 = 57_600;

const GATEWAY_IP: &str = "192.168.1.1";
const LOCAL_IP: &str = "192.168.1.110";
const DEST_IP: &str = "192.168.1.100";

const ANGLE_REPORT_INTERVAL_MS: u128 = 1000 / 60; // 60Hz

// Used for looking up device number
const MAC_ADDRESS_LIST: [[u8; 6]; 7] = [
    [0xfc, 0xb4, 0x67, 0xcf, 0x14, 0x34],
    [0x24, 0xdc, 0xc3, 0xd0, 0x24, 0x44],
    [0xfc, 0xb4, 0x67, 0xce, 0x0a, 0xc8],
    [0xfc, 0xb4, 0x67, 0xcd, 0xd9, 0x04],
    [0xfc, 0xb4, 0x67, 0xda, 0xdb, 0x40],
    [0xfc, 0xb4, 0x67, 0xda, 0xd8, 0xd0],
    [0xfc, 0xb4, 0x67, 0xcf, 0x0c, 0x00],
];

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();

    esp_idf_svc::log::EspLogger::initialize_default();

    unsafe {
        esp_idf_sys::nvs_flash_init();
    }
    let _nvs = EspDefaultNvsPartition::take().unwrap();
    let sysloop = EspSystemEventLoop::take().unwrap();

    // GPIO configs ////////////////////////////////////////////////////////////////////////////

    // Pin Config
    let peripherals = Peripherals::take().unwrap();

    let mut channel_laser_red = LedcDriver::new(
        peripherals.ledc.channel0,
        LedcTimerDriver::new(
            peripherals.ledc.timer0,
            &ledc_config::TimerConfig::new().frequency(25.kHz().into()),
        )?,
        peripherals.pins.gpio15,
    )?;
    channel_laser_red.set_duty(0)?;

    let mut channel_laser_green = LedcDriver::new(
        peripherals.ledc.channel1,
        LedcTimerDriver::new(
            peripherals.ledc.timer1,
            &ledc_config::TimerConfig::new().frequency(25.kHz().into()),
        )?,
        peripherals.pins.gpio13,
    )?;
    channel_laser_green.set_duty(0)?;

    let mut channel_laser_blue = LedcDriver::new(
        peripherals.ledc.channel2,
        LedcTimerDriver::new(
            peripherals.ledc.timer2,
            &ledc_config::TimerConfig::new().frequency(25.kHz().into()),
        )?,
        peripherals.pins.gpio14,
    )?;
    channel_laser_blue.set_duty(0)?;

    let mut emac_pwr = PinDriver::output(peripherals.pins.gpio4)?;
    emac_pwr.set_high()?;

    /////////////////////////////// Dynamixel TTL Comm Config //////////////////////////////////////////////
    let mut dxl_sel = PinDriver::output(peripherals.pins.gpio32)?;
    // let mut dxl_sel = PinDriver::output(peripherals.pins.gpio14)?;
    let dxl_driver = {
        // Working gpio32
        // GPIO35: Unable to use for output
        // gpio36 sensor_v
        // gpio39 sensor_vn
        let tx = peripherals.pins.gpio33;
        // let tx = peripherals.pins.gpio32;
        let rx = peripherals.pins.gpio35;
        // let tx = peripherals.pins.gpio15;
        // let rx = peripherals.pins.gpio13;
        let config = uart_config::Config::new()
            .baudrate(Hertz(DXL_BAUDRATE))
            .data_bits(uart_config::DataBits::DataBits8)
            .parity_none()
            .stop_bits(uart_config::StopBits::STOP1)
            .flow_control(uart_config::FlowControl::None);

        // UartDriver::new(
        AsyncUartDriver::new(
            peripherals.uart1,
            tx,
            rx,
            Option::<gpio::Gpio0>::None,
            Option::<gpio::Gpio1>::None,
            &config,
        )?
        // dxl_driver
    };

    dxl_sel.set_low()?;

    /////////////////////////////// Ethernet Config //////////////////////////////////////////////
    // Get device No according to the efuse mac address
    let mut mac: [u8; 6] = [0u8; 6];
    let mac_addr_ptr: *mut u8 = mac.as_mut_ptr();
    let res = unsafe { esp_idf_sys::esp_base_mac_addr_get(mac_addr_ptr) };
    let dev_no = if res == esp_idf_sys::ESP_OK {
        info!("MAC: {:x?}", mac);
        lookup_device_no(&mac).unwrap_or_default()
        // if let Ok(n) = lookup_device_no(&mac) {
        //     n
        // } else {
        //     0u8
        // }
    } else {
        0u8
    };

    // Ethernet Config
    let local_ip = Ipv4Addr::from_str(LOCAL_IP)?;
    let ip = local_ip.octets();
    let local_ip = Ipv4Addr::new(ip[0], ip[1], ip[2], ip[3] + dev_no);

    let gateway_ip = Ipv4Addr::from_str(GATEWAY_IP)?;
    let dest_ip = Ipv4Addr::from_str(DEST_IP)?;

    let res = {
        // Hoping to start up the RMMII
        // delay::Ets::delay_ms(2u32);
        // delay::Ets::delay_ms(10u32);

        let res = esp_idf_svc::eth::EthDriver::new_rmii(
                peripherals.mac,
                peripherals.pins.gpio25,
                peripherals.pins.gpio26,
                peripherals.pins.gpio27,
                peripherals.pins.gpio23,
                peripherals.pins.gpio22,
                peripherals.pins.gpio21,
                peripherals.pins.gpio19,
                peripherals.pins.gpio18,
                esp_idf_svc::eth::RmiiClockConfig::<gpio::Gpio0, gpio::Gpio16, gpio::Gpio17>::OutputGpio0(
                    peripherals.pins.gpio0,
                ),
                Some(peripherals.pins.gpio5),
                esp_idf_svc::eth::RmiiEthChipset::LAN87XX,
                // Phy address must be assigned, you can't leave it to None
                // None,
                Some(0),
                sysloop.clone(),
            );
        res
    };

    let eth_driver = match res {
        Ok(res) => {
            info!("Eth driver init done");
            res
        }
        Err(e) => {
            error!("Eth driver init failed: {e}");
            let _ = reboot_after_message(1000);
            // Unreachable
            unreachable!();
        }
    };

    let mut eth_config = netif::NetifConfiguration::eth_default_client();
    let ipconfig = ipv4::Configuration::Client(ClientConfiguration::Fixed(ClientSettings {
        ip: local_ip,
        subnet: Subnet {
            gateway: gateway_ip,
            mask: Mask(24),
        },
        dns: None,
        secondary_dns: None,
    }));

    eth_config.ip_configuration = ipconfig;
    let res = netif::EspNetif::new_with_conf(&eth_config);
    let eth_netif: netif::EspNetif = match res {
        Ok(res) => {
            info!("Eth netif init done");
            res
        }
        Err(e) => {
            error!("Eth netif init failed: {e}");
            let _ = reboot_after_message(1);
            unreachable!();
        }
    };

    // let mac = esp_idf_svc::netif::EspNetif::get_mac(&eth_netif);
    // if let Ok(m) = mac {
    //     info!("MAC: {:x?}", m);
    // };

    let mut eth: Box<EspEth<'_, RmiiEth>> =
        Box::new(esp_idf_svc::eth::EspEth::wrap_all(eth_driver, eth_netif)?);

    let res = eth_configure(&sysloop, &mut eth);
    let _eth_ip_info = match res {
        Ok(res) => {
            info!("Eth configure done");
            res
        }
        Err(e) => {
            error!("Eth configure failed: {e}");
            let _ = reboot_after_message(1000);
            unreachable!();
        }
    };

    // LED indicator config //////////////////////////////////////////////////////////////
    let led_pin = peripherals.pins.gpio12;
    let led_chan = peripherals.rmt.channel0;
    let ws2812 = Ws2812Esp32Rmt::new(led_chan, led_pin).unwrap();

    info!("Software Version: {}", SOFTWARE_VERSION);

    /////////////////////////////// Communication Buffers ////////////////////////////////

    // Thread communication buffers!
    let (upstream_msg_producer, upstream_msg_consumer) = QUEUE_UPSTREM.try_split_framed().unwrap();
    let (downstream_msg_producer, downstream_msg_consumer) =
        QUEUE_DOWNSTREAM.try_split_framed().unwrap();

    // Dynamixel queue, to the upstream, PC via OSC
    let (dxl_upstream_msg_producer, dxl_upstream_msg_consumer) =
        QUEUE_DXL_UPSTREAM.try_split_framed().unwrap();
    // To the oriental motor via dynamixel
    let (dxl_downstream_msg_producer, dxl_downstream_msg_consumer) =
        QUEUE_DXL_DOWNSTREAM.try_split_framed().unwrap();

    /////////////////////////////// Threads ////////////////////////////////
    // OSC downstream receiver thread
    let osc_receiver_join_handle =
        std::thread::Builder::new()
            .stack_size(8192)
            .spawn(move || {
                let mut osc = OscReceiver::new(
                    local_ip,
                    LOCAL_OSC_RECV_PORT,
                    downstream_msg_producer,
                    dxl_downstream_msg_producer,
                );
                loop {
                    // let ts = std::time::SystemTime::now();

                    if let Err(e) = osc.run() {
                        error!("Failed to run OSC: {e}");
                        // break;
                    }
                    // info!("OSC RX loop: {:?}us", ts.elapsed().unwrap().as_micros());
                    osc.idle();
                }
            })?;

    // OSC upstream sender thread
    let osc_sender_join_handle = std::thread::Builder::new()
        .stack_size(8192)
        .spawn(move || {
            let mut osc_sender = OscSender::new(
                dest_ip,
                REMOTE_OSC_PORT,
                local_ip,
                LOCAL_OSC_SEND_PORT,
                upstream_msg_consumer,
                dxl_upstream_msg_consumer,
                dev_no,
            );
            osc_sender.send_bootmsg().unwrap();
            loop {
                // let ts = std::time::SystemTime::now();

                if let Err(e) = osc_sender.run() {
                    error!("Failed to run OSC Sender: {e}");
                    // break;
                }
                // info!("OSC RX loop: {:?}us", ts.elapsed().unwrap().as_micros());
                osc_sender.idle();
            }
        })?;

    // Dynamixel communication thread to Oriental motors
    let dxl_join_handle = std::thread::Builder::new()
        .stack_size(12288)
        .spawn(move || {
            let mut dxl = Dynamixel::new(
                dxl_upstream_msg_producer,
                dxl_downstream_msg_consumer,
                dxl_driver,
                dxl_sel,
            );

            dxl.init().unwrap();

            loop {
                // let ts = std::time::SystemTime::now();
                if let Err(e) = dxl.update() {
                    error!("Failed to handle dxl communication: {e}");
                    // break;
                }
                // info!("Mod loop: {:?}us", ts.elapsed().unwrap().as_micros());

                dxl.idle();
            }
        })?;

    // peripheral control
    let peripherals_join_handle =
        std::thread::Builder::new()
            .stack_size(6144)
            .spawn(move || {
                let mut device_manager = Device::new(
                    upstream_msg_producer,
                    downstream_msg_consumer,
                    ws2812,
                    channel_laser_red,
                    channel_laser_green,
                    channel_laser_blue,
                );

                device_manager.init().unwrap();

                loop {
                    // let ts = std::time::SystemTime::now();

                    let _ = device_manager.update();
                    // info!("Dev loop: {:?}us", ts.elapsed().unwrap().as_micros());
                    std::thread::sleep(GPIO_SLEEP_DURATION);

                    // let res = ping(gateway_ip);
                    // if let Err(e) = res {
                    //     error!("Ping failed: {e}");
                    // }
                    // std::thread::sleep(Duration::from_secs(5));
                }
            })?;

    osc_sender_join_handle.join().unwrap();
    osc_receiver_join_handle.join().unwrap();
    peripherals_join_handle.join().unwrap();

    dxl_join_handle.join().unwrap();

    Ok(())
}

fn eth_configure<T>(
    sysloop: &EspSystemEventLoop,
    eth: &mut esp_idf_svc::eth::EspEth<T>,
) -> Result<IpInfo> {
    info!("Eth created");
    let mut eth = esp_idf_svc::eth::BlockingEth::wrap(eth, sysloop.clone())?;
    eth.start()?;

    info!("Waiting for netif up...");

    eth.wait_netif_up()?;

    let ip_info = eth.eth().netif().get_ip_info()?;

    info!("Eth info: {:?}", ip_info);
    Ok(ip_info)
}

#[allow(dead_code)]
fn ping(ip: ipv4::Ipv4Addr) -> Result<()> {
    info!("About to do some pings for {:?}", ip);

    let ping_summary = ping::EspPing::default().ping(ip, &Default::default())?;
    if ping_summary.transmitted != ping_summary.received {
        bail!("Pinging IP {} resulted in timeouts", ip);
    }
    info!("Pinging done");
    Ok(())
}

pub fn reboot_after_message(delay_ms: u32) -> Result<()> {
    error!("Rebooting in {} seconds...", delay_ms / 1000);
    delay::Ets::delay_ms(delay_ms);
    unsafe {
        esp_idf_sys::esp_restart();
    }
    // Ok(())
}

fn lookup_device_no(mac: &[u8; 6]) -> Result<u8> {
    let mut device_no = 1u8;
    let mut device_no_found = false;
    for m in MAC_ADDRESS_LIST.iter() {
        if m == mac {
            device_no_found = true;
            break;
        }
        device_no += 1;
    }

    if !device_no_found {
        device_no = 0;
    }
    info!("Device No: {}", device_no);

    Ok(device_no)
}
