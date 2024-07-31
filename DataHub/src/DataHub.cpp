#include <vector>
#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include "DebugSupport.h"

#define MAC_SIZE 6
#define PAIRING_PIN 4

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

const uint8_t ESP_OUI[] = {0x18, 0xFE, 0x34};
uint8_t sender_address[MAC_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

enum runing_state {LISTENING, PAIRING, NORMAL, NODATA, ERROR};

runing_state current_state = NODATA;
runing_state previous_state = ERROR;
uint16_t og_millis = 0;
int rssi = 0;

typedef struct struct_message {
  runing_state operation;
  uint16_t adc_value;
  uint8_t heartBeat;
} struct_message;

typedef struct saved_values{
  uint8_t mac[MAC_SIZE];
  struct_message data;
} saved_values;

typedef struct saved_paired_device{
  uint8_t mac[MAC_SIZE];
} saved_paired_device;

std::vector<saved_values> received_data;
std::vector<saved_paired_device> paired_devices;

void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
  if (type != WIFI_PKT_MGMT)
    return;

  static const uint8_t ACTION_SUBTYPE = 0xd0;

  typedef struct {
    unsigned frame_ctrl: 16;  
    unsigned duration_id: 16;
    uint8_t receiver_addr[6];
    uint8_t sender_addr[6]; 
    uint8_t filtering_addr[6];
    unsigned sequence_ctrl: 16;
    unsigned category:8;
    uint8_t addr4[6]; // Contains organizationally Unique Identifier
  } wifi_ieee80211_mac_hdr_t;

  typedef struct {
    wifi_ieee80211_mac_hdr_t hdr;
    uint8_t payload[0]; // network data ended with 4 bytes csum (CRC32) 
  } wifi_ieee80211_packet_t;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  // Only continue processing if this is an action frame containing the Espressif OUI.
  if ((ACTION_SUBTYPE == (hdr->frame_ctrl & 0xFF)) && (memcmp(hdr->addr4, ESP_OUI, 3) == 0)){
    rssi = ppkt->rx_ctrl.rssi;
    saved_paired_device device;
    memcpy(&device.mac[0], hdr->sender_addr, MAC_SIZE);
    
    //TODO: This is a debug print, remove it later
    printMacAddress(Serial, &device.mac[0], "Sender MAC address: ");

    Serial.print("RSSI: ");
    Serial.println(rssi);
    //end of debug print

    if(rssi < -30 || sizeof(paired_devices) > 19 ){
      return;
    }
    
    bool new_device = true;
    if(sizeof(paired_devices) == 0)
    {
      memcpy(&sender_address, &device.mac[0], sizeof(sender_address));
      Serial.println("First device found");
      current_state = PAIRING;
    }
    else{
      for (auto saved_device : paired_devices){
        if(memcmp(&device.mac[0], &saved_device, sizeof(sender_address)) == 0){
          new_device = false;
          break;
        }
      }
    }
    if(new_device){
      memcpy(&sender_address, &device.mac[0], sizeof(sender_address));
      Serial.println("New device found");
      current_state = PAIRING;
    }
  }
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  uint8_t macOfSender[6];
  struct_message received_message;
  memcpy(&macOfSender, mac, sizeof(macOfSender));
  memcpy(&received_message, incomingData, sizeof(received_message));  

  if((current_state == NORMAL || current_state == NODATA) && received_message.operation == NORMAL){    //TODO: Acept just if paired before
    if(received_data.size() == 0){
      bool accepted_device = false;
      for(auto saved_device : paired_devices){
        if(memcmp(&saved_device.mac[0], &macOfSender[0], sizeof(macOfSender)) == 0){
          accepted_device = true;
          break;
        }
      }
      if(accepted_device){
        saved_values new_data;
        memcpy(&new_data.data, incomingData, sizeof(new_data.data));  
        memcpy(&new_data.mac, mac, sizeof(macOfSender));
        received_data.push_back(new_data);
      }
    }
    else{
      bool new_element = true;
      for(int i = 0; i < received_data.size(); i++) {
        if(memcmp(&received_data[i].mac, mac, sizeof(macOfSender)) == 0){
          memcpy(&received_data[i].data, incomingData, sizeof(received_data[i].data));
          new_element = false;
          break;
        }
      }
      if(new_element){
        bool accepted_device = false;
        for(auto saved_device : paired_devices){
          if(memcmp(&saved_device.mac[0], &macOfSender[0], sizeof(macOfSender)) == 0){
            accepted_device = true;
            break;
          }
        }
        if(accepted_device){
          saved_values new_data;
          memcpy(&new_data.data, incomingData, sizeof(new_data.data));
          memcpy(&new_data.mac, mac, sizeof(macOfSender));
          received_data.push_back(new_data);
        }
      }
    }
  }
  else if(current_state == PAIRING && received_message.operation == PAIRING){
    Serial.println("Pairing confirmed");
    saved_paired_device device;
    memcpy(&device.mac[0], mac, MAC_SIZE);
    paired_devices.push_back(device);
    current_state = NORMAL;
  }
  else{
    Serial.printf("Something did not match: Operation is %d and message operation is %d\n", current_state, received_message.operation);
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup(void) {
  // Init Serial Monitor
  Serial.begin(115200);

  // Init OLED
  u8g2.begin();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  pinMode(PAIRING_PIN, INPUT);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv/sender callbacks to get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(OnDataSent);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);

  //Start of Pairing for esp8266 devices //TODO: Remove this part when ESP8266 is not used anymore
  esp_now_peer_info_t peerInfo={};
  saved_paired_device temp_device;

  uint8_t temp_address[MAC_SIZE] = {0x48, 0xE7, 0x29, 0x6D, 0x38, 0xDC};
    // Register peer
  memcpy(peerInfo.peer_addr, temp_address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
    // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(&temp_device.mac[0], &temp_address[0], MAC_SIZE);
  paired_devices.push_back(temp_device);
  
  uint8_t second_temp_address[MAC_SIZE] = {0xEC, 0x64, 0xC9, 0xC5, 0x17, 0x4C};
  // Register peer
  memcpy(peerInfo.peer_addr, second_temp_address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(&temp_device.mac[0], &second_temp_address[0], MAC_SIZE);
  paired_devices.push_back(temp_device);

  uint8_t third_temp_address[MAC_SIZE] = {0xEC, 0x64, 0xC9, 0xC5, 0x05, 0x13};
  // Register peer
  memcpy(peerInfo.peer_addr, third_temp_address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(&temp_device.mac[0], &third_temp_address[0], MAC_SIZE);
  paired_devices.push_back(temp_device);
  //End of Pairing for esp8266 devices
}

void loop(void) {
    esp_err_t pairing_send_result = ESP_OK;
    
    switch (current_state){
    case PAIRING:
      static int pairing_counter = 0;
      struct_message pairing_data;
      esp_now_peer_info_t peerInfo;
      
      if(current_state != previous_state){
        Serial.println("Just entered pairing state");
        previous_state = current_state;
        
        WiFi.mode(WIFI_MODE_STA);
        esp_wifi_set_promiscuous(0);
        
        // Register peer
        memcpy(peerInfo.peer_addr, sender_address, 6);
        peerInfo.channel = 0;  
        peerInfo.encrypt = false;
          // Add peer        
        if (esp_now_add_peer(&peerInfo) != ESP_OK){
          Serial.println("Failed to add peer");
          return;
        }
      }

      // Send message via ESP-NOW
      pairing_data.operation = PAIRING;
      pairing_send_result = esp_now_send(sender_address, (uint8_t *) &pairing_data, sizeof(pairing_data));      
      Serial.print("Pairing send result: ");
      Serial.println(pairing_send_result);
      printMacAddress(Serial, &sender_address[0], "Pairing with device: ");

      displayBigText(&u8g2, "Pairing");
      delay(100);

      pairing_counter++;
      if(pairing_counter > 100){ //Timeout for pairing
        pairing_counter = 0;
        esp_now_del_peer(peerInfo.peer_addr);
        current_state = NORMAL;
      }
      break;

    case LISTENING:
      static int listening_counter = 0;
      if(current_state != previous_state){
        Serial.println("Just entered listening state");
        previous_state = current_state;
        
        WiFi.mode(WIFI_MODE_APSTA);
        esp_wifi_set_promiscuous(1);
      }

      displayBigText(&u8g2, "Listening");
      delay(100);

      listening_counter++;
      if(listening_counter > 100){
        listening_counter = 0;
        current_state = NORMAL;
      }
      break;

    case NORMAL:
      static int update_counter = 0;
      if(current_state != previous_state){
        Serial.println("Just entered normal state");
        previous_state = current_state;

        WiFi.mode(WIFI_STA);

        esp_wifi_set_promiscuous(0);

        for (auto saved_device : paired_devices){  // Print all paired devices debug TODO: Remove
          printMacAddress(Serial, &saved_device.mac[0], "Paired device MAC Address: ");
        }

      }

      if (digitalRead(PAIRING_PIN) == HIGH){
        current_state = LISTENING;
      }
      else{
        if(received_data.size() == 0){
          displayBigText(&u8g2, "No data");
          
          if(paired_devices.size() == 0){
            current_state = NODATA;
          }
        }
        else{
          if(update_counter % 30 != 0){
            u8g2.clearBuffer();
          }
          else{
            saved_values received = received_data[int(update_counter / 30)];
            printMacAddress(Serial, &received.mac[0], "On screen now: ");
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_u8glib_4_tf);
            u8g2.drawStr(20,17,"Sensor number");
            u8g2.drawUTF8(90,17,u8x8_u16toa(received.data.heartBeat,3));
            u8g2.drawUTF8(110,17,u8x8_u16toa(received.mac[5],3));
            u8g2.setFont(u8g2_font_8x13B_tr);
            u8g2.drawUTF8(50,32,u8x8_u16toa(received.data.adc_value,4));
            u8g2.sendBuffer();
            }
        }

        update_counter++;
        if (update_counter > 30 * received_data.size() - 1){
          update_counter = 0;
        }
        delay(100);
      }
      break;
    
    case NODATA:
      if(current_state != previous_state){
        Serial.println("Just entered no data state");
        previous_state = current_state;
        displayBigText(&u8g2, "No data");
      }

      if (digitalRead(PAIRING_PIN) == HIGH){
        current_state = LISTENING;
      }

      delay(100);

      if(received_data.size() > 0){
        current_state = NORMAL;
      }
      break;

    default:{
      Serial.println("Error: Invalid state");
      break;
    }
      
  }
}