#include <Arduino.h>
#include <U8g2lib.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define MAX_DEVICES 5
#define interval 30000

BLEScan* pBLEScan;
BLEAddress* knownDevices[MAX_DEVICES] = {nullptr};
int deviceCount = 0;
int sensorValue = 0;
unsigned long previousMillis = 0;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
        BLEAddress deviceAddress = advertisedDevice.getAddress();
        bool known = false;
        for (int i = 0; i < deviceCount; i++) {
          if (knownDevices[i] != nullptr && *knownDevices[i] == deviceAddress) {
            known = true;
            break;
          }
        }
        if (!known && deviceCount < MAX_DEVICES) {
          knownDevices[deviceCount++] = new BLEAddress(deviceAddress);
        }
      }
    }
};

void setup(void) {
  Serial.begin(115200);
  u8g2.begin();

  BLEDevice::init("ESP32 Central");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  previousMillis = millis();
}

void loop(void) {
  
  unsigned long currentMillis = millis();
  // If the interval has passed, update the characteristic
  if (currentMillis - previousMillis >= interval) {
  
    previousMillis = currentMillis;
    Serial.println("Scanning...");
    BLEScanResults foundDevices = pBLEScan->start(3, false);
    Serial.print("Devices found: ");
    Serial.println(foundDevices.getCount());
    pBLEScan->clearResults();
  }

  for (int i = 0; i < deviceCount; i++) {
    if (knownDevices[i] == nullptr) continue;
    
    Serial.println("Connecting to device...");
    Serial.println(knownDevices[i]->toString().c_str());
    
    BLEClient* pClient = BLEDevice::createClient();
    if (pClient->connect(*knownDevices[i])) {
      BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
      if (pRemoteService != nullptr) {
        BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
        if (pRemoteCharacteristic != nullptr) {
          std::string value = pRemoteCharacteristic->readValue();
          
          if (value.length() == 4) {
            // Convert byte array to integer
            sensorValue = ((uint8_t)value[0] << 24) | ((uint8_t)value[1] << 16) | ((uint8_t)value[2] << 8) | (uint8_t)value[3];
            Serial.printf("Sensor %s value: %d\n", knownDevices[i]->toString().c_str(), sensorValue);
          } else {
            Serial.println("Received value is not of expected length.");
          }
        }
      }
      pClient->disconnect();
    } else {
      Serial.println("Failed to connect");
    }
    delay(100);  // Short delay between connection attempts
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_u8glib_4_tf);
  u8g2.drawStr(20,17,"Sensor number");
  u8g2.drawUTF8(90,17,u8x8_u16toa(0,2));
  u8g2.drawUTF8(110,17,u8x8_u16toa(1,2));
  u8g2.setFont(u8g2_font_8x13B_tr);
  u8g2.drawUTF8(50,32,u8x8_u16toa(sensorValue,4));
  u8g2.sendBuffer();

  delay(5000);  // Reduced delay to 5 seconds
}