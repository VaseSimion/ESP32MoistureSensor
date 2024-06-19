# ESP Moisture Sensor

This repository contains the source code for a project intended to gather moisture sensor data from a mesh of ESP8266/ESP32.

## Overview

The program reads a value from a moisture sensor and sends it through ESP-NOW. This is done on the sender end. On the receiver end the program monitors this and sends it to a place where it can be analyzed and processed (this part is not yet implemented).  

## Files

## Setup

1. Clone this repository.
2. Open the project in your preferred IDE (e.g., Visual Studio Code, PlatformIO).
3. Build and upload the code to your ESP8266/ESP32 device.

## Usage
Once the code is uploaded to the ESP8266 device, it will start reading the moisture sensor value and send it through ESP-NOW. The receiver part will display the received data on an OLED display.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change. Please make sure to update tests as appropriate.

## License

[MIT](https://choosealicense.com/licenses/mit/)
