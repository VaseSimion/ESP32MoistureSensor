# ESP Moisture Sensor

This repository contains the source code for a moisture sensor using ESP8266/ESP32.

## Overview

The program reads a value from a moisture sensor and sends it through ESP-NOW. It can also receive values from ESP-NOW but currently ignores them.

## Files

- `Esp8266Main.cpp`: This is the main file that contains the setup and loop functions. It initializes the ESP-NOW, registers the send and receive callbacks, and reads the moisture sensor value in the loop.
- `Receiver.cpp`: This file contains the code for the receiver part of the ESP-NOW communication. It receives the data sent by the sender, stores it, and displays it on an OLED display. This is currently done for ESP32
- `Receiver.ino`: This file is similar to `Receiver.cpp` but is designed to be used with the Arduino IDE. It is written specifically for ESP32.
- `Main.cpp`: This file contains the main function of the program. It initializes the ESP8266 device and starts the main loop.
- `Main.ino`: This file is similar to `Main.cpp` but is designed to be used with the Arduino IDE.

## Setup

1. Clone this repository.
2. Open the project in your preferred IDE (e.g., Visual Studio Code, PlatformIO).
3. Build and upload the code to your ESP8266 device.

## Usage

Once the code is uploaded to the ESP8266 device, it will start reading the moisture sensor value and send it through ESP-NOW. The receiver part will display the received data on an OLED display.

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change. Please make sure to update tests as appropriate.

## License

[MIT](https://choosealicense.com/licenses/mit/)
