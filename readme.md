# About
- Rust project for the NSK 2024 angle encoder
## Versions
0. Angle and speed encoder, /catch, /release in ms order
1. Modbus communication implementation. WIP.
2. Modbus write working with AsyncUart. Reading is not implemented.
3. Added 100 us delay

## Target
- Target Hardware [T-Internet-POE](https://www.lilygo.cc/products/t-internet-poe)
- xtensa-esp32-espidf

## Project
The project is written on Rust for EPS32.

## Sensor
[AS5600 data sheet](https://www.mouser.com/pdfdocs/AMS_AS5600_Datasheet_EN.PDF)

## Environment
- Windows 11 / DevContainer
- Target ESP-IDF v5.1.1

# Setting up environment

## Configs

## Build Commands
```PowerShell
~/export-esp.ps1
cargo run
espflash board-info
espmonitor <OCM_PORT_NO>
espflash flash  --monitor -p=/dev/ttyACM1 ./bin/encoderosc
```

```Bash
. ~/export-esp.sh
espmonitor /dev/ttyACM1
```

### Current workflow using DevContainer
```DevContainer
cargo build
web-flash --chip esp32 target/xtensa-esp32-espidf/debug/encoderosc

# Move the binary to windows side (From ubuntu terminal)
cp target/xtensa-esp32-espidf/debug/spektra_shutl_laser /mnt/c/Users/donne/Desktop

# Then flash from powershell (Windows side)
espflash flash --baud 2000000 encoderosc
```

## Test Build
cargo run --features test

### Build in offline mode
```
cargo build --offline
```

# Protocol
[Protocol List](https://docs.google.com/spreadsheets/d/1DaNTpB7jWGoRZkjAx4mv4fX9doqdj3NTubLn8uKJP9o/edit?usp=sharing)

## How to test
```
oscd
```

### Commands

## Crate
- Using [bbqueue](https://docs.rs/bbqueue/latest/bbqueue/) for between threads communiations.
- [WS2812 Driver](https://github.com/cat-in-136/ws2812-esp32-rmt-driver/tree/main)
- [CRC16](https://docs.rs/crc16/latest/crc16/)

## References
- [Rust-ESP32-STD-demo](https://github.com/ivmarkov/rust-esp32-std-demo/blob/main/src/main.rs)
- [ESP-IDF-SVC ethernet sample](https://github.com/esp-rs/esp-idf-svc/blob/master/examples/eth.rs)
- [ESP-IDF-HAL-LEDC](https://github.com/esp-rs/esp-idf-hal/blob/master/examples/ledc_threads.rs)
## TODO
[x] Get speed
[x] PWM solenoid
[] Modbus
[] Catch, release in us order