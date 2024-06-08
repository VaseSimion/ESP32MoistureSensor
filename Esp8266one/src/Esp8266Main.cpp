#include <ESP8266WiFi.h>
#include <espnow.h>
#include "Arduino.h"

#define SENDER_ID 1
//This reads the value and sends it through ESP now and it's done for ESP8266
//It gets values from ESP Now but it ignores them for the moment

const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
const int sensorPowerPin = 4; // ESP8266 Digital Pin D2 = GPIO4
int adcValueRead = 0;

uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xD4, 0x50};
String success;

typedef struct message
{
  uint16_t idOfOgSender;
  uint16_t adcValue;
  uint8_t heartBeat;
} message_struct;

uint16_t ogSender = 0;
uint16_t adcValue = 0;
uint8_t heartBeat = 0;
message_struct receivedData;
message_struct sendData;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("\r\nLast Packet Send Status:\t");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  ogSender = receivedData.idOfOgSender;
  adcValue = receivedData.adcValue;
  heartBeat = receivedData.heartBeat;
}
void setup(void) {
    // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  sendData.idOfOgSender = SENDER_ID;
  sendData.heartBeat = 0;
  pinMode(sensorPowerPin, OUTPUT);
}

void loop(void) {
  digitalWrite(sensorPowerPin, HIGH);
  delay(100);
  adcValueRead = analogRead(analogInPin);
  digitalWrite(sensorPowerPin, LOW);
  sendData.adcValue = 4*adcValueRead;
  sendData.heartBeat++;
  Serial.println(4*adcValueRead);
  Serial.print("Value gotten from the other:");
  Serial.println(adcValue);
    delay(500);
    // Send message via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));
  delay(900);
}

