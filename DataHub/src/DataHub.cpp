#include <Arduino.h>
#include <U8g2lib.h> // from https://github.com/olikraus/u8g2
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "LittleFS.h"
#include <ESP32Time.h> // from https://github.com/fbiego/ESP32Time

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define NUMBER_OF_SENDERS 2
#define MAC_SIZE 6

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

uint8_t broadcastAddress[] = {0x10, 0x06, 0x1C, 0xA6, 0xCF, 0x74};
String success;

ESP32Time rtc(3600);  // offset in seconds GMT+1

typedef struct message
{
  uint8_t macOfOgSender[MAC_SIZE];
  uint16_t adcValue;
  uint8_t heartBeat;
} message_struct;

message_struct receivedData;
message_struct sendData;
message_struct stored_receivedData[NUMBER_OF_SENDERS];

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
  for(int i = 0; i < MAC_SIZE; i++){
    Serial.print(receivedData.macOfOgSender[i], HEX);
    if(i < MAC_SIZE - 1)
    {
      Serial.print(":");
    }
  }
  Serial.println();
  
  memcpy(&stored_receivedData[0], &receivedData, sizeof(receivedData));
  memcpy(&stored_receivedData[1], &receivedData, sizeof(receivedData));

  Serial.print("Heartbeat: ");
  Serial.println(receivedData.heartBeat);
}

void setup(void) {
  // Init Serial Monitor
  Serial.begin(115200);

  u8g2.begin();
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  if(!LittleFS.begin(true)){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  

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

  esp_wifi_get_mac(WIFI_IF_STA, sendData.macOfOgSender);

  rtc.setTime(00, 24, 12, 22, 6, 2024);  // 17th Jan 2021 15:24:30
}

void loop(void) {
  static int i = 1;
  static int sensor_id = 0;
  

  File file = LittleFS.open("/text.txt");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  
  Serial.println("File Content:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();

  Serial.println(rtc.getTime());          //  (String) 15:24:38

  sendData.adcValue = i;
  i++;
  sensor_id++;
  if(sensor_id > NUMBER_OF_SENDERS - 1)
  {
    sensor_id = 0;
  }

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  {
    u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFont(u8g2_font_u8glib_4_tf );	// choose a suitable font
    u8g2.drawStr(20,17,"Sensor number");	// write something to the internal memory

    u8g2.drawUTF8(90,17,u8x8_u16toa(stored_receivedData[sensor_id].macOfOgSender[0],2));	// write something to the internal memory

    u8g2.drawUTF8(110,17,u8x8_u16toa(stored_receivedData[sensor_id].heartBeat,2));	// write something to the internal memory

    u8g2.setFont(u8g2_font_8x13B_tr);	// choose a suitable font
    u8g2.drawUTF8(50,32,u8x8_u16toa(stored_receivedData[sensor_id].adcValue,4));	// write something to the internal memory

  /*  if(stored_receivedData[sensor_id].adcValue < 1200)
    {
        u8g2.drawStr(12,32,"Not connected");	// write something to the internal memory
    }
    else if(stored_receivedData[sensor_id].adcValue < 1600)
    {
        u8g2.drawStr(12,32,"Pure water");	// write something to the internal memory
    }
    else if(stored_receivedData[sensor_id].adcValue < 1900)
    {
        u8g2.drawStr(12,32,"Just watered it");	// write something to the internal memory
    }
    else if(stored_receivedData[sensor_id].adcValue < 2200)
    {
        u8g2.drawStr(12,32,"Could drink");	// write something to the internal memory
    }
    else if(stored_receivedData[sensor_id].adcValue < 2500)
    {
        u8g2.drawStr(12,32,"Thirsty");	// write something to the internal memory
    }
    else
    {
        u8g2.drawStr(12,32,"Dry AF");	// write something to the internal memory
    }
*/
  }
  //u8g2.drawUTF8(32,32,u8x8_u16toa(adcValue,4));	// write something to the internal memory
  
  u8g2.sendBuffer();					// transfer internal memory to the display
  delay(2000);  
}

