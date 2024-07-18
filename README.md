# BLE Sensor Network

This repository contains the source code for a simple BLE (Bluetooth Low Energy) sensor network using ESP32. The network is divided into two main components: DataSender and DataHub.

## DataSender.cpp

This component is responsible for reading sensor data and sending it over BLE. It initializes a BLE server on an ESP32 device, advertises a custom service and characteristic, and updates the characteristic with sensor data at regular intervals.

Key Features:
- Initializes and starts a BLE server with a custom service and characteristic.
- Reads sensor data and updates the BLE characteristic.
- Handles BLE connections and disconnections.

## DataHub.cpp

This component acts as a central device that scans for, connects to, and reads data from BLE peripheral devices advertising the custom service and characteristic defined in DataSender.

Key Features:
- Scans for BLE devices advertising the custom service.
- Connects to devices and reads the advertised custom characteristic.
- Displays the sensor data on an OLED display connected to the ESP32.

## Hardware Requirements

- Two ESP32 development boards
- Analog sensor (for DataSender, simulated by ADC pin)
- SSD1306 OLED display (for DataHub)

## Software Dependencies

- Arduino IDE
- ESP32 BLE Arduino library
- U8g2 library for OLED display

## Setup and Running

1. **DataSender Setup:**
   - Connect the analog sensor to pin 34 of the ESP32.
   - Flash `DataSender.cpp` to the ESP32 device.

2. **DataHub Setup:**
   - Connect the OLED display to the ESP32 using I2C.
   - Flash `DataHub.cpp` to a second ESP32 device.

3. Power both ESP32 devices. The DataHub will start scanning for DataSender devices and display the sensor data on the OLED screen.

## License

This project is licensed under the MIT License - see the LICENSE file for details.