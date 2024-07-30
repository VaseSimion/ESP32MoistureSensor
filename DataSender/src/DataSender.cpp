#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <LittleFS.h>

#define ADC_PIN 34
#define MAC_SIZE 6
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  3        /* Time ESP32 will go to sleep (in seconds) */

// REPLACE WITH YOUR RECEIVER MAC Address
//uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xD4, 0x50};
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xCE, 0x3C}; //sender
enum running_state {LISTENING, PAIRING, NORMAL, NODATA, ERROR};

uint16_t adc_value = 0;
uint16_t og_millis = 0;

typedef struct struct_message {
  running_state operation;
  uint16_t adc_value;
  uint8_t heartBeat;
} struct_message;

// Create a struct_message called myData
struct_message myData;
running_state local_operation = LISTENING;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    static uint8_t failure_count = 0;
    Serial.print("Last Packet Send Status: ");
    Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    Serial.print(" to MAC address: ");
    for(int i = 0; i < MAC_SIZE; i++){
      Serial.print(mac_addr[i], HEX);
      Serial.print(":");
    }
    Serial.println();

    if(status == ESP_NOW_SEND_SUCCESS && local_operation==PAIRING){
      File file = LittleFS.open("/opMode.txt", FILE_WRITE);
      local_operation = NORMAL;
      file.print((uint8_t)local_operation);
      file.close();
      file = LittleFS.open("/broadcastMac.txt", FILE_WRITE);
      for(int i = 0; i < MAC_SIZE; i++){
        file.printf("%02x",mac_addr[i]);
      }
      file.close();
      delay(100);
    }
    else if(local_operation == NORMAL && status == ESP_NOW_SEND_SUCCESS){
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
    else if(local_operation == NORMAL && status != ESP_NOW_SEND_SUCCESS )
    {
      failure_count++;
      Serial.print("Failure count: ");
      Serial.println(failure_count);
      if(failure_count > 3){
        local_operation = LISTENING;
        File file = LittleFS.open("/opMode.txt", FILE_WRITE);
        file.print((uint8_t)local_operation);
        file.close();
        failure_count = 0;
      }
      delay(1000);
    }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  uint8_t macOfSender[6];
  struct_message received_message;
  memcpy(&macOfSender, mac, sizeof(macOfSender));
  memcpy(&received_message, incomingData, sizeof(received_message));  
  
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

  if(local_operation == LISTENING && received_message.operation == PAIRING){    
    local_operation = PAIRING;
    memcpy(&broadcastAddress, mac, sizeof(macOfSender));
    Serial.println("Pairing with new device");
    Serial.println("New MAC address: ");
    for(int i = 0; i < MAC_SIZE; i++){
      Serial.print(broadcastAddress[i], HEX);
      Serial.print(":");
    }
    Serial.println();

    esp_now_del_peer(peerInfo.peer_addr);

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
  }
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
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  if(!LittleFS.begin(true)){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  File file = LittleFS.open("/opMode.txt");
  if(!file){
    Serial.println("Failed to read the operation mode");
    local_operation = LISTENING;
  }
  else{
    Serial.println("Operation mode file opened successfully");
    String content = file.readString();
    local_operation = (running_state)content.toInt();
    file.close();
  }

  Serial.print("Booting operation mode: ");
  Serial.println(local_operation);

  file = LittleFS.open("/heartbeat.txt");
  if(!file){
    Serial.println("Failed to open file for reading");
    myData.heartBeat = 0;
  }
  else{
    String content = file.readString();
    file.close();
    myData.heartBeat = content.toInt();
  }
  
  // Print the current heartbeat value
  Serial.print("Current heartbeat: ");
  Serial.println(myData.heartBeat);
  
  // Increment the heartbeat
  myData.heartBeat++;
  
  // Save the new heartbeat value
  file = LittleFS.open("/heartbeat.txt", "w");
  if(!file){
    Serial.println("Failed to open file for writing");
    myData.heartBeat = 0;
    file.print(myData.heartBeat);
  }
  else{
    file.print(myData.heartBeat);
    file.close();
  }
  
  if(!LittleFS.exists("/opMode.txt")){
    Serial.println("opMode.txt does not exist");
    local_operation = LISTENING;
    file = LittleFS.open("/opMode.txt", FILE_WRITE);
    file.print((uint8_t)local_operation);
    file.close();
  }

  if(!LittleFS.exists("/broadcastMac.txt")){
    Serial.println("broadcastMac.txt does not exist");
    file = LittleFS.open("/broadcastMac.txt", FILE_WRITE);
    file.print(0);
    file.close();
  }

  if(local_operation != LISTENING){
    file = LittleFS.open("/broadcastMac.txt");
    if(!file){
      Serial.println("Failed to read the broadcast MAC address");
      uint8_t local[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      memcpy(&broadcastAddress[0], &local[0], 6);
    }
    else{
      String content = file.readString();
      Serial.print("Broadcast string MAC address: ");
      Serial.println(content);
      file.close();
      for(int i = 0; i < MAC_SIZE; i++){
        if(isDigit(content[2*i])){
          broadcastAddress[i] = content[2*i] - '0';
        }
        else{
          broadcastAddress[i] = content[2*i] - 'a' + 10;
        }
        if(isDigit(content[2*i+1])){
          broadcastAddress[i] = (broadcastAddress[i] << 4) | (content[2*i+1] - '0');
        }
        else{
          broadcastAddress[i] = (broadcastAddress[i] << 4) | (content[2*i+1] - 'a' + 10);
        }
      }
    }
  }
  
  Serial.print("Booted with Broadcast MAC address: ");
  for(int i = 0; i < MAC_SIZE; i++){
    Serial.print(broadcastAddress[i], HEX);
    Serial.print(":");
  }
  Serial.println();
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  esp_err_t send_result;
  switch(local_operation){
    case LISTENING:
    case PAIRING:
      myData.operation = local_operation;
      send_result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      delay(1000);
      break;
    case NORMAL:
      delay(150);
        // Set values to send
      myData.operation = NORMAL;
      myData.adc_value = analogRead(ADC_PIN);
      myData.heartBeat++;
      // Send message via ESP-NOW
      send_result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
      
      if (send_result == ESP_OK) {
          Serial.println("Sent with success");
      }
      else {
          Serial.println("Error sending the data");
          delay(1000);
      }
      break;
    case NODATA:
      
      break;
    case ERROR:
      // Error state
      break;
  }

}