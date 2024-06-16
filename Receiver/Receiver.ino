#include <Arduino.h>
#include <U8g2lib.h> // from https://github.com/olikraus/u8g2
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

//uint8_t broadcastAddress[] = {0x10, 0x06, 0x1C, 0xA6, 0xCF, 0x74};
uint8_t broadcastAddress[] = {0xEC, 0x64, 0xC9, 0xC4, 0xFF, 0x01};

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
void OnDataRecv(const esp_now_recv_info_t *esp_now_recv_cb, const uint8_t *incomingData, int len) {
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

  u8g2.begin();
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
  
 // esp_wifi_set_protocol( WIFI_IF_AP, WIFI_PROTOCOL_LR );
  
  sendData.heartBeat = 0;
  sendData.idOfOgSender = 9;
}

void loop(void) {
  static int i = 1;
  sendData.adcValue = i;
  sendData.heartBeat++;
  i++;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_u8glib_4_tf );	// choose a suitable font
  u8g2.drawStr(40,17,"ADC Value");	// write something to the internal memory

  u8g2.drawUTF8(100,17,u8x8_u16toa(ogSender,2));	// write something to the internal memory

  u8g2.setFont(u8g2_font_8x13B_tr);	// choose a suitable font
  if(adcValue < 1600)
  {
      u8g2.drawStr(12,32,"Pure water");	// write something to the internal memory
  }
  else if(adcValue < 1900)
  {
      u8g2.drawStr(12,32,"Just watered it");	// write something to the internal memory
  }
  else if(adcValue < 2200)
  {
      u8g2.drawStr(12,32,"Could drink a bit");	// write something to the internal memory
  }
  else if(adcValue < 2500)
  {
      u8g2.drawStr(12,32,"Thirsty");	// write something to the internal memory
  }
  else
  {
      u8g2.drawStr(12,32,"Dry AF");	// write something to the internal memory
  }

  
  //u8g2.drawUTF8(32,32,u8x8_u16toa(adcValue,4));	// write something to the internal memory

  
  u8g2.sendBuffer();					// transfer internal memory to the display
  delay(1000);  
}

