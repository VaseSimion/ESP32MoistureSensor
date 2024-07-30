#include <vector>
#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

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
    
    //TODO: This is a debug print, remove it later, also the memcpy should be removed
    Serial.print("Sender MAC address: ");
    for(int i = 0; i < MAC_SIZE; i++){
      Serial.print(device.mac[i], HEX);
      Serial.print(":");
    }
    Serial.println();
    Serial.print("RSSI: ");
    Serial.println(rssi);
    //end of debug print

    bool new_device = true;
    //TODO Implement the rssi check and the max number of devices allowed
    if(sizeof(paired_devices) == 0)
    {
      memcpy(&sender_address, &device.mac[0], sizeof(sender_address));
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
  
  /*
  // Print the incoming data for debugging purposes
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Operation: ");
  Serial.println(received_message.operation);
  Serial.print("ADC value: ");
  Serial.println(received_message.adc_value);
  Serial.print("Heartbeat: ");
  Serial.println(received_message.heartBeat);
  Serial.print("MAC address: ");
  for(int i = 0; i < MAC_SIZE; i++){
    Serial.print(macOfSender[i], HEX);
    Serial.print(":");
  }
  Serial.println();
  // end of debug print
  */
  if(current_state == NORMAL && received_message.operation == NORMAL){    //TODO: Acept just if paired before
    if(received_data.size() == 0){
      saved_values new_data;
      memcpy(&new_data.data, incomingData, sizeof(new_data.data));  
      memcpy(&new_data.mac, mac, sizeof(macOfSender));
      received_data.push_back(new_data);
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
        saved_values new_data;
        memcpy(&new_data.data, incomingData, sizeof(new_data.data));
        memcpy(&new_data.mac, mac, sizeof(macOfSender));
        received_data.push_back(new_data);
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
  else if (current_state == NODATA && received_message.operation == NORMAL)
  {
    current_state = NORMAL; //TODO: This is temporary until pariring works
  }
  else{
    Serial.println("Something does not match");
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
      Serial.print("Pairing with device: ");
      for(int i = 0; i < MAC_SIZE; i++){
        Serial.print(sender_address[i], HEX);
        Serial.print(":");
      }
      Serial.println();

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_8x13B_tr);
      u8g2.drawStr(40,25,"Pairing");
      u8g2.sendBuffer();
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

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_8x13B_tr);
      u8g2.drawStr(40,25,"Listening");
      u8g2.sendBuffer();
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
         Serial.print("Paired device MAC address: ");
          for(int i = 0; i < MAC_SIZE; i++){
            Serial.print(saved_device.mac[i], HEX);
            Serial.print(":");
          }
          Serial.println();
        }

      }

      if (digitalRead(PAIRING_PIN) == HIGH){
        current_state = LISTENING;
      }
      else{
        if(received_data.size() == 0){
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_u8glib_4_tf);
          u8g2.drawStr(20,17,"No data received");
          u8g2.sendBuffer();
        }
        else{
          if(update_counter % 3 != 0){
            u8g2.clearBuffer();
          }
          else{
            saved_values received = received_data[int(update_counter / 3)];
            Serial.print("MAC address: ");
            for(int i = 0; i < MAC_SIZE; i++){
              Serial.print(received.mac[i], HEX);
              Serial.print(":");
            }
            Serial.println();
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
        if (update_counter > 3 * received_data.size() - 1){
          update_counter = 0;
        }
        delay(1000);
      }
      break;
    
    case NODATA:
      if(current_state != previous_state){
        Serial.println("Just entered no data state");
        previous_state = current_state;
      }

      if (digitalRead(PAIRING_PIN) == HIGH){
        current_state = LISTENING;
      }

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_8x13B_tr);
      u8g2.drawStr(40,32,"No data");
      u8g2.sendBuffer();
      delay(1000);
    
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