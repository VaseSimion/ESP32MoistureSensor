# ESP8266 and ESP32 Data Communication Project

This project demonstrates how to set up communication between ESP8266 and ESP32 devices using ESP-NOW. The project includes three main components:

- **SimpleESP8266Sender.cpp**: A simple sender script for ESP8266.
- **DataSender.cpp**: A more advanced sender script for ESP32.
- **DataHub.cpp**: A receiver script for ESP32 that collects and displays data from multiple senders.

## Components

### SimpleESP8266Sender.cpp

This script is designed for the ESP8266 and sends data using ESP-NOW. It includes a callback function to handle the status of sent data.

### DataSender.cpp

This script is designed for the ESP32 and includes additional functionality such as deep sleep and file system operations. It reads a heartbeat value from a file, increments it, and sends it along with other data using ESP-NOW.

### DataHub.cpp

This script is designed for the ESP32 and acts as a data hub, receiving data from multiple senders. It stores the data and displays it on an OLED screen.

## Setup Instructions

### Hardware Requirements

- ESP8266 and ESP32 development boards
- OLED display for the ESP32 data hub
- Necessary wiring and power supply

### Software Requirements

- Arduino IDE or PlatformIO
- ESP8266 and ESP32 board support packages
- Required libraries: `WiFi`, `esp_now`, `LittleFS`, `U8g2lib`

### Installation

1. **Clone the repository:**

    ```sh
    git clone https://github.com/VaseSimion/ESP32MoistureSensor
    cd ESP32MoistureSensor
    ```

2. **Open the project in your preferred IDE (Arduino IDE or PlatformIO).**

3. **Install the required libraries:**

    - For Arduino IDE, use the Library Manager to install `WiFi`, `esp_now`, `LittleFS`, and `U8g2lib`.
    - For PlatformIO, add the libraries to your `platformio.ini` file.

4. **Upload the scripts to the respective devices:**

    - `SimpleESP8266Sender.cpp` to the ESP8266.
    - `DataSender.cpp` to the ESP32 sender.
    - `DataHub.cpp` to the ESP32 receiver.

### Configuration

1. **Set the MAC addresses:**

    - Update the `broadcastAddress` array in `DataSender.cpp` and `DataHub.cpp` with the MAC addresses of your devices.

2. **Configure the OLED display:**

    - Ensure the correct pins are defined for the OLED display in `DataHub.cpp`.

### Running the Project

1. **Power on the ESP8266 and ESP32 devices.**
2. **Monitor the Serial output to verify data transmission and reception.**
3. **Observe the OLED display on the ESP32 data hub for received data.**

### Troubleshooting

- Ensure all devices are powered and connected properly.
- Verify the MAC addresses are correctly set.
- Check the Serial Monitor for any error messages.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments

Thanks to the developers of the ESP-NOW protocol and the Arduino community for their support and contributions.

For more information, visit the [official documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html).