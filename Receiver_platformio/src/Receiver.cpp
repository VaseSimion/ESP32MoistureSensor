#include <Arduino.h>
#include <U8g2lib.h> // from https://github.com/olikraus/u8g2
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define NUMBER_OF_SENDERS 2

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

//uint8_t broadcastAddress[] = {0x10, 0x06, 0x1C, 0xA6, 0xCF, 0x74};
uint8_t broadcastAddress[] = {0xEC, 0x64, 0xC9, 0xC5, 0x0A, 0x75};
String success;
int rssi = 0;

typedef struct message
{
  uint16_t idOfOgSender;
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
  stored_receivedData[receivedData.idOfOgSender] = receivedData;
}

void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {

    // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
    if (type != WIFI_PKT_MGMT)
        return;

    static const uint8_t ACTION_SUBTYPE = 0xd0;
    static const uint8_t ESPRESSIF_OUI[] = {0xEC, 0x64, 0xC9};
    
    typedef struct {
      unsigned frame_ctrl: 16;
      unsigned duration_id: 16;
      uint8_t receiver_addr[6];
      uint8_t sender_oui[3]; // Organizationally Unique Identifier
      uint8_t sender_uaa[3]; // Universally Administred Address
      uint8_t filtering_addr[6];
      unsigned sequence_ctrl: 16;
      uint8_t addr4[6]; // optional
    } wifi_ieee80211_mac_hdr_t;

    typedef struct {
      wifi_ieee80211_mac_hdr_t hdr;
      uint8_t payload[0]; // network data ended with 4 bytes csum (CRC32) 
    } wifi_ieee80211_packet_t;

    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    // Only continue processing if this is an action frame containing the Espressif OUI.
    if ((ACTION_SUBTYPE == (hdr->frame_ctrl & 0xFF)) &&
        (memcmp(hdr->sender_oui, ESPRESSIF_OUI, 3) == 0)) {
        rssi = ppkt->rx_ctrl.rssi;
        Serial.print("RSSI: ");
        Serial.println(rssi);
     }
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
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
  sendData.idOfOgSender = 2;
}

void loop(void) {
  static int i = 1;
  static int sensor_id = 0;
  
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

    u8g2.drawUTF8(90,17,u8x8_u16toa(stored_receivedData[sensor_id].idOfOgSender,2));	// write something to the internal memory

    u8g2.drawUTF8(110,17,u8x8_u16toa(stored_receivedData[sensor_id].heartBeat,2));	// write something to the internal memory

    u8g2.setFont(u8g2_font_8x13B_tr);	// choose a suitable font
    //u8g2.drawUTF8(50,32,u8x8_u16toa(stored_receivedData[sensor_id].adcValue,4));	// write something to the internal memory

    char str_rssi[4];
    sprintf(str_rssi, "%d", rssi);
    u8g2.drawUTF8(50,32,str_rssi);	// write something to the internal memory

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

