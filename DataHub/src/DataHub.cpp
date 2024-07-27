#include <vector>
#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#define MAC_SIZE 6

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

// uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xD4, 0x50};
// uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xCE, 0x3C}; //theother ESP32
enum runing_state {PAIRING, NORMAL, NODATA, ERROR};
runing_state current_state = NODATA;
runing_state previous_state = ERROR;
uint16_t og_millis = 0;

typedef struct struct_message {
  char name[32];
  uint16_t adc_value;
  uint8_t heartBeat;
} struct_message;

typedef struct saved_values{
  uint8_t mac[MAC_SIZE];
  struct_message data;
} saved_values;

std::vector<saved_values> received_data;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  uint8_t macOfSender[6];
  memcpy(&macOfSender, incomingData, sizeof(macOfSender));
  
  if(received_data.size() == 0){
    saved_values new_data;
    memcpy(&new_data.data, incomingData, sizeof(new_data.data));  
    memcpy(&new_data.mac, &macOfSender, sizeof(macOfSender));
    received_data.push_back(new_data);
  }
  else{
    bool new_element = true;
    for(int i = 0; i < received_data.size(); i++) {
      if(memcmp(&received_data[i].mac, &macOfSender, sizeof(macOfSender)) == 0){
        memcpy(&received_data[i].data, incomingData, sizeof(received_data[i].data));
        new_element = false;
        break;
      }
    }
    if(new_element){
      saved_values new_data;
      memcpy(&new_data.data, incomingData, sizeof(new_data.data));
      memcpy(&new_data.mac, &macOfSender, sizeof(macOfSender));
      received_data.push_back(new_data);
    }
  }
}

void setup(void) {
  // Init Serial Monitor
  Serial.begin(115200);

  // Init OLED
  u8g2.begin();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop(void) {
    switch (current_state){
    case PAIRING:
      if(current_state != previous_state){
        Serial.println("Just entered pairing state");
        previous_state = current_state;
      }
      Serial.println("Pairing");
      current_state = NORMAL; //currently not using pairing
      break;

    case NORMAL:
      if(current_state != previous_state){
        Serial.println("Just entered normal state");
        previous_state = current_state;
      }

      if(received_data.size() == 0){
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_u8glib_4_tf);
        u8g2.drawStr(20,17,"No data received");
        u8g2.sendBuffer();
        delay(3000);
      }

      for(auto received: received_data){
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_u8glib_4_tf);
        u8g2.drawStr(20,17,"Sensor number");
        u8g2.drawUTF8(90,17,u8x8_u16toa(received.data.heartBeat,3));
        u8g2.drawUTF8(110,17,u8x8_u16toa(received.mac[5],3));
        u8g2.setFont(u8g2_font_8x13B_tr);
        u8g2.drawUTF8(50,32,u8x8_u16toa(received.data.adc_value,4));
        u8g2.sendBuffer();
        delay(3000);
      }
      break;
    
    case NODATA:
      if(current_state != previous_state){
        Serial.println("Just entered no data state");
        previous_state = current_state;
      }

      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_8x13B_tr);
      u8g2.drawStr(40,32,"No data");
      u8g2.sendBuffer();
      delay(3000);
    
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