#include <Arduino.h>
#include "NimBLEDevice.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define ADC_PIN 34

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  3        /* Time ESP32 will go to sleep (in seconds) */

uint8_t adcValueBytes[4] = {0, 0, 0, 0};
NimBLECharacteristic *pCharacteristic = nullptr;
bool isNotified = false;
uint16_t ogmillis = 0;

class MyCallbacks: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
        Serial.print("Characteristic read by client: ");
        Serial.println(pCharacteristic->getUUID().toString().c_str());
        isNotified = true;
        Serial.println("Going to sleep now");
        Serial.println("Time elapsed: ");
        Serial.println(millis() - ogmillis);
    }
};

void setup() {
    ogmillis = millis();
  Serial.begin(115200);
    NimBLEDevice::init("ESP32 Sensor");
    
    NimBLEServer *pServer = NimBLEDevice::createServer();
    NimBLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE |
                                               NIMBLE_PROPERTY::NOTIFY
                                              );
    
    pCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();
    pCharacteristic->setValue(adcValueBytes, 4);

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID); 
    pAdvertising->start(); 
}

void loop() {
    if( isNotified == false)
    {
        delay(300);
        long freeMem = ESP.getFreeHeap(); Serial.print("Remaining memory "); Serial.println(freeMem);
        // Read ADC value
        int adcValue = analogRead(ADC_PIN);
        
        // Convert integer to byte array
        uint8_t adcValueBytes[4];
        adcValueBytes[0] = (adcValue >> 24) & 0xFF;
        adcValueBytes[1] = (adcValue >> 16) & 0xFF;
        adcValueBytes[2] = (adcValue >> 8) & 0xFF;
        adcValueBytes[3] = adcValue & 0xFF;

        // Update characteristic value
        pCharacteristic->setValue(adcValueBytes, 4);
        pCharacteristic->notify();
        
        Serial.print("ADC Value: ");
        Serial.println(adcValue);
    }
    else
    {   
        esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
        esp_deep_sleep_start();
    }
}