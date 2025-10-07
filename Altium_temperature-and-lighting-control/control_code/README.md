# ESP32-S3 PID Temperature Control System

## Overview
This project is a temperature control system based on the ESP32-S3, featuring:
- PID temperature control
- OLED display for real-time status
- Five rotary encoders for RGB, white brightness, and target temperature adjustment
- Five NeoPixel LED strips (20 LEDs each)
- Heating and cooling relay control
- BMP3XX temperature and pressure sensor

## Hardware Connections
- **BMP3XX Sensor**: I2C (SCL: GPIO19, SDA: GPIO20, CS: GPIO18)
- **OLED Display**: I2C (Address: 0x3C)
- **Relays**:
  - Heating: GPIO12
  - Cooling: GPIO11
- **Rotary Encoders**:
  - Red: CLK GPIO15, DT GPIO16
  - Green: CLK GPIO6, DT GPIO7
  - Blue: CLK GPIO17, DT GPIO18
  - Temperature: CLK GPIO9, DT GPIO10
  - White: CLK GPIO4, DT GPIO5
- **NeoPixel LED Strips**:
  - Pin 1: GPIO35 (RGB)
  - Pin 2: GPIO36 (White)
  - Pin 3: GPIO37 (RGB)
  - Pin 4: GPIO38 (White)
  - Pin 5: GPIO39 (RGB)
  - Each strip: 20 LEDs

## Features
- **PID Control**: Maintains the target temperature using heating/cooling relays.
- **Rotary Encoders**:
  - Adjust Red, Green, Blue, White brightness, and target temperature.
- **NeoPixel Control**:
  - Strips 1, 3, 5: RGB color
  - Strips 2, 4: White brightness
- **OLED Display**:
  - Shows current/target temperature, PID output, RGB/white values, relay and LED status.
- **Serial Output**:
  - Prints all key status and debug information.

## Usage
1. Connect all hardware as described above.
2. Upload the code using PlatformIO.
3. Use the rotary encoders to adjust color, white brightness, and target temperature.
4. Monitor the OLED and serial output for real-time feedback.

## File Structure
- `src/main.cpp`: Main application code
- `platformio.ini`: PlatformIO project configuration
- `README.md`: Project documentation

## Dependencies
- Adafruit BMP3XX Library
- Adafruit SSD1306 Library
- Adafruit NeoPixel Library

Install all dependencies via PlatformIO Library Manager.

## License
MIT
