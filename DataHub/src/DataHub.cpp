#include <Arduino.h>
#include <U8g2lib.h>
#include "NimBLEDevice.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


int sensorValue = 0;
unsigned long previousMillis = 0;



void setup(void) {
  Serial.begin(115200);
  u8g2.begin();
  NimBLEDevice::init("ESP HUB");
}

void loop(void) {
  
  long freeMem = ESP.getFreeHeap(); Serial.print("Remaining memory "); Serial.println(freeMem);
  unsigned long currentMillis = millis();
  NimBLEScan *pScan = NimBLEDevice::getScan();
  NimBLEScanResults results = pScan->start(1);

  NimBLEUUID serviceUuid(SERVICE_UUID);
  
  for(int i = 0; i < results.getCount(); i++) {
      NimBLEAdvertisedDevice device = results.getDevice(i);
      
      if (device.isAdvertisingService(serviceUuid)) {
          NimBLEClient *pClient = NimBLEDevice::createClient();
          
          if (pClient->connect(&device)) {
              NimBLERemoteService *pService = pClient->getService(serviceUuid);
              
              if (pService != nullptr) {
                  NimBLERemoteCharacteristic *pCharacteristic = pService->getCharacteristic(CHARACTERISTIC_UUID);
                  
                  if (pCharacteristic != nullptr) {
                      std::string value = pCharacteristic->readValue();
        
                      if (value.length() == 4) {
                        // Convert byte array to integer
                        sensorValue = ((uint8_t)value[0] << 24) | ((uint8_t)value[1] << 16) | ((uint8_t)value[2] << 8) | (uint8_t)value[3];
                        Serial.printf("Sensor %s value: %d\n", device.getAddress().toString().c_str(), sensorValue);
                      } else {
                        Serial.println("Received value is not of expected length.");
                      }
                      // print or do whatever you need with the value
                  }
              }
          } else {
          // failed to connect
          }
          NimBLEDevice::deleteClient(pClient);
      }
  }

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_u8glib_4_tf);
  u8g2.drawStr(20,17,"Sensor number");
  u8g2.drawUTF8(90,17,u8x8_u16toa(0,2));
  u8g2.drawUTF8(110,17,u8x8_u16toa(1,2));
  u8g2.setFont(u8g2_font_8x13B_tr);
  u8g2.drawUTF8(50,32,u8x8_u16toa(sensorValue,4));
  u8g2.sendBuffer();

  delay(1000);  // Reduced delay to 5 seconds
}