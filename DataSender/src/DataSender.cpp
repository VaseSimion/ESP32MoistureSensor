#include <Arduino.h>
#include "NimBLEDevice.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define ADC_PIN 34

uint8_t adcValueBytes[4] = {0, 0, 0, 0};
NimBLECharacteristic *pCharacteristic = nullptr;


class MyCallbacks: public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic){
        Serial.print("Characteristic read by client: ");
        Serial.println(pCharacteristic->getUUID().toString().c_str());
    }
};


void setup() {
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

    delay(1000);
}