#include <esp_now.h>
#include <WiFi.h>
#include "Arduino.h"

//This reads the value and sends it through ESP now
//It gets values from ESP Now but it ignores them for the moment

const int sensorInputPin = 33;
int adcValueRead = 0;

uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xD4, 0x50};
String success;

typedef struct message
{
  uint16_t idOfOgSender;
  uint16_t adcValue;
} message_struct;

uint16_t ogSender = 0;
uint16_t adcValue = 0;

message_struct receivedData;
message_struct sendData;
esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t *esp_now_recv_cb, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  ogSender = receivedData.idOfOgSender;
  adcValue = receivedData.adcValue;
}


void setup(void) {
    // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  sendData.idOfOgSender = 1;
}

void loop(void) {
  adcValueRead = analogRead(sensorInputPin);
  sendData.adcValue = adcValueRead;
  Serial.println(adcValue);
    // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));

  delay(1000);
}

