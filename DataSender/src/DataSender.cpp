#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <LittleFS.h>

#define ADC_PIN 34

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  3        /* Time ESP32 will go to sleep (in seconds) */

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xD4, 0x50};
// uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xCE, 0x3C}; //sender

uint16_t adc_value = 0;
uint16_t og_millis = 0;

typedef struct struct_message {
  char name[32];
  uint16_t adc_value;
  uint8_t heartBeat = 0;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    Serial.println("Going to sleep now");
    Serial.println("Time: " + String(millis() - og_millis));
    og_millis = millis();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
}

void setup() {
  og_millis = millis();
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

  if(!LittleFS.begin(true)){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  File file = LittleFS.open("/heartbeat.txt");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  String content = file.readString();
  file.close();
  
  myData.heartBeat = content.toInt();
  
  // Print the current heartbeat value
  Serial.print("Current heartbeat: ");
  Serial.println(myData.heartBeat);
  
  // Increment the heartbeat
  myData.heartBeat++;
  
  // Save the new heartbeat value
  file = LittleFS.open("/heartbeat.txt", "w");
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  
  if(file.print(myData.heartBeat)){
    Serial.println("Heartbeat updated successfully");
  } else {
    Serial.println("Failed to update heartbeat");
  }
  file.close();

  file.close();
}

void loop() {
    delay(150);
      // Set values to send
    strcpy(myData.name, "Esp Sender");
    myData.adc_value = analogRead(ADC_PIN);
    myData.heartBeat++;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    
    if (result == ESP_OK) {
        Serial.println("Sent with success");
    }
    else {
        Serial.println("Error sending the data");
    }
}