#include <ESP8266WiFi.h>
#include <espnow.h>
#include "Arduino.h"
#include "LittleFS.h"

#define SENDER_ID 0
#define MAC_SIZE 6
//This reads the value and sends it through ESP now and it's done for ESP8266
//It gets values from ESP Now but it ignores them for the moment

const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
const int sensorPowerPin = 12; // ESP8266 Digital Pin D6 = GPIO12

uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xD4, 0x50};
enum runing_state {PAIRING, NORMAL, NODATA, ERROR};

typedef struct struct_message {
  runing_state operation;
  uint16_t adc_value;
  uint8_t heartBeat;
} struct_message;

static struct_message sendData;

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

/* This is the data that will be received from the other ESP8266 device
* Currently, it is not being used

uint8_t macOgSender[MAC_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint16_t adcValue = 0;
uint8_t heartBeat = 0;
message_struct receivedData;
// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  memcpy(&macOgSender, receivedData.macOfOgSender, sizeof(macOgSender));
  adcValue = receivedData.adcValue;
  heartBeat = receivedData.heartBeat;
}
*/

void setup(void) {
  int time = millis();
  // Init the GPIO
  pinMode(D0, WAKEUP_PULLUP);
  pinMode(sensorPowerPin, OUTPUT);
  digitalWrite(sensorPowerPin, HIGH);
  
  ESP.rtcUserMemoryRead(0, (uint32_t *) &sendData, sizeof(sendData));
  sendData.operation = NORMAL;
    // Init Serial Monitor
  Serial.begin(115200);
  Serial.setTimeout(2000);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  //Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  //esp_now_register_recv_cb(OnDataRecv);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  /*Initialize the LittleFS file system
  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  File file = LittleFS.open("/paired.txt", "r");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
    Serial.println("File Content:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
  
*/

/*
  wifi_get_macaddr(STATION_IF, sendData.macOfOgSender);
  for(int i = 0; i < MAC_SIZE; i++){
    Serial.print(sendData.macOfOgSender[i], HEX);
    if(i < MAC_SIZE - 1)
    {
      Serial.print(":");
    }
  }
  Serial.println();
*/

  sendData.adc_value = 4*analogRead(analogInPin);
  sendData.heartBeat ++;
//  delay(30);
  digitalWrite(sensorPowerPin, LOW);
    // Send message via ESP-NOW
  ESP.rtcUserMemoryWrite(0, (uint32_t *) &sendData, sizeof(sendData));
  esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));


  Serial.print("Time taken to send data: ");
  Serial.println(millis() - time);

  //delay(10000);

  ESP.deepSleep(10e6);
}

void loop(void) {
}

