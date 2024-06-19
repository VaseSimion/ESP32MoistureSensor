# README.md

## Project Overview

This project consists of three main components, each represented by a C++ file: `SimpleESP8266Sender.cpp`, `DataRepeater.cpp`, and `DataHub.cpp`.

### SimpleESP8266Sender.cpp

This file contains the code for a simple sender device using the ESP8266 microcontroller. The sender sends data packets containing an ID, an ADC value, and a heartbeat signal. The sender also has callback functions for when data is sent and received. The data sent status is printed to the serial monitor.

### DataRepeater.cpp

The `DataRepeater.cpp` file is responsible for receiving data from the sender, and then re-transmitting that data. It also contains callback functions for when data is sent and received. The status of the data sent is stored in a string variable and the received data is stored in a struct.

### DataHub.cpp

The `DataHub.cpp` file represents the main hub in the network. It receives data from the repeater and stores it in an array of structs. The hub also sends its own data packets, with the ADC value incrementing each time a packet is sent (for the moment as a placeholder). The hub uses a U8g2 library to display the received data on an SSD1306 OLED display.

## Setup

To set up this project, you will need to install the U8g2 libraries in your platformio in Vscode. You will also need to configure the broadcast addresses in each file to match your network setup.

## Usage

Upload the corresponding file to each ESP8266 device in your network. The sender should be running `SimpleESP8266Sender.cpp`, the repeater should be running `DataRepeater.cpp`, and the hub should be running `DataHub.cpp`. Once the devices are powered on and within range of each other, they will begin transmitting and receiving data.

## .Ino files
There are 2 arduino files (.ino) which are used for testing and fast prototyping
