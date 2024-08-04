#include <string.h>
#include <string>
#include <vector>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_littlefs.h"
#include "esp_sleep.h"

//Defines
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define MAC_SIZE 6
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  30       /* Time ESP32 will go to sleep (in seconds) */

//Logging tags
static const char* TAG = "General";

//Adc variables
static int adc_raw[2][10];
adc_oneshot_unit_handle_t adc_handle;

//Enums
enum running_state {LISTENING, PAIRING, NORMAL, NODATA, ERROR};

//Structs
struct struct_message {
  running_state operation;
  uint16_t adc_value;
  uint8_t heartBeat;

    struct_message(running_state o = NORMAL, uint16_t a = 55, uint8_t h = 96)
        : operation(o), adc_value(a), heartBeat(h) {}
};

// Create a struct_message called myData
struct_message myData;
running_state local_operation = LISTENING;
esp_now_peer_info_t peerInfo;

//MAC addresses
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t general_broadcast_mac[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
//uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xD4, 0x50};

//Function prototypes
bool write_uint8_array_to_file(const char* filename, const uint8_t* data, size_t length);

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Last Packet Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    ESP_LOGI(TAG, "Sent to MAC Address: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    static uint8_t failure_count = 0;

    if(status == ESP_NOW_SEND_SUCCESS && local_operation==PAIRING){
        write_uint8_array_to_file("/storage/target.bin", mac_addr, 6);
        local_operation = NORMAL;
        write_uint8_array_to_file("/storage/opMode.bin", (uint8_t*)&local_operation, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    else if(local_operation == NORMAL && status == ESP_NOW_SEND_SUCCESS){
      ESP_LOGI(TAG, "Should be foing to sleep here");
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
    else if(local_operation == NORMAL && status != ESP_NOW_SEND_SUCCESS )
    {
        failure_count++;
        ESP_LOGI(TAG, "Failure count: %d", failure_count);
        if(failure_count > 3){
            local_operation = LISTENING;
            write_uint8_array_to_file("/storage/opMode.bin", (uint8_t*)&local_operation, 1);
            memcpy(&broadcastAddress[0], &general_broadcast_mac[0], 6);

            esp_now_del_peer(peerInfo.peer_addr);
            memcpy(peerInfo.peer_addr, &general_broadcast_mac[0], 6);
            peerInfo.channel = 0;  
            peerInfo.encrypt = false;
            if (esp_now_add_peer(&peerInfo) != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add peer");
                return;
            }
            failure_count = 0;
        }
    }
}

// Updated callback function with robust MAC address printing
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
    uint8_t macOfSender[MAC_SIZE];
    struct_message received_message;
    memcpy(&macOfSender, esp_now_info->src_addr, sizeof(macOfSender));
    memcpy(&received_message, incomingData, sizeof(received_message));  
    ESP_LOGI(TAG, "Received from: %02X:%02X:%02X:%02X:%02X:%02X", macOfSender[0], macOfSender[1], macOfSender[2], macOfSender[3], macOfSender[4], macOfSender[5]);

    if(local_operation == LISTENING && received_message.operation == PAIRING){    
        local_operation = PAIRING;
        memcpy(&broadcastAddress, esp_now_info->src_addr, sizeof(macOfSender));
        ESP_LOGI(TAG, "Pairing with: %02X:%02X:%02X:%02X:%02X:%02X", broadcastAddress[0], broadcastAddress[1], broadcastAddress[2], broadcastAddress[3], broadcastAddress[4], broadcastAddress[5]);

        esp_now_del_peer(peerInfo.peer_addr);
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        peerInfo.channel = 0;  
        peerInfo.encrypt = false;
        
        // Add peer    
        memcpy(peerInfo.peer_addr, broadcastAddress, 6);
        peerInfo.channel = 0;  
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add peer");
            return;
        }
    }
}

void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void init_esp_now() {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(OnDataSent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(OnDataRecv));
}

void init_adc(adc_oneshot_unit_handle_t *padc_handle){
    adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, padc_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*padc_handle, ADC_CHANNEL_0, &config));
    //For calibration check https://github.com/espressif/esp-idf/blob/v5.3/examples/peripherals/adc/oneshot_read/main/oneshot_read_main.c
}

// LittleFS functions
void init_littlefs() {
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/storage",
        .partition_label = "storage",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };
    
    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}

bool write_uint8_array_to_file(const char* filename, const uint8_t* data, size_t length) {
    //ESP_LOGI(TAG, "Writing uint8_t array to file: %s", filename);
    FILE* f = fopen(filename, "wb");  // Note the "wb" for writing in binary mode
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return false;
    }
    size_t written = fwrite(data, sizeof(uint8_t), length, f);
    if (written != length) {
        ESP_LOGE(TAG, "Failed to write to file: wrote %d of %d bytes", written, length);
        fclose(f);
        return false;
    }
    fclose(f);
    return true;
}

std::vector<uint8_t> read_uint8_array_from_file(const char* filename) {
    //ESP_LOGI(TAG, "Reading uint8_t array from file: %s", filename);
    FILE* f = fopen(filename, "rb");  // Note the "rb" for reading in binary mode
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return std::vector<uint8_t>();
    }
    
    // Get file size
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    // Read file contents into vector
    std::vector<uint8_t> data(fsize);
    size_t read = fread(data.data(), sizeof(uint8_t), fsize, f);
    if (read != fsize) {
        ESP_LOGE(TAG, "Failed to read from file: read %d of %d bytes", read, (int)fsize);
    }
    fclose(f);
    
    return data;
}

void init_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

}


extern "C" void app_main(void)
{
    init_nvs();
    wifi_init();
    init_esp_now();
    init_littlefs();
    init_adc(&adc_handle);


    {
        std::vector<uint8_t> read_data = read_uint8_array_from_file("/storage/opMode.bin");
        if (!read_data.empty()) {
            local_operation = (running_state)read_data[0];
            ESP_LOGI(TAG, "Operation mode: %d", local_operation);
        } else {
            local_operation = LISTENING;
            write_uint8_array_to_file("/storage/opMode.bin", (uint8_t*)&local_operation, 1);
            ESP_LOGE(TAG, "Failed to read operation mode from file");
        }    
    }

    {
        std::vector<uint8_t> read_data = read_uint8_array_from_file("/storage/heartbeat.bin");
        if (!read_data.empty()) {
            myData.heartBeat = read_data[0];
            ESP_LOGI(TAG, "Heartbeat: %d", myData.heartBeat);
        } else {
            myData.heartBeat = 91;
            ESP_LOGE(TAG, "Failed to read heartbeat from file");
        }    
    }
    myData.heartBeat++;
    {
        write_uint8_array_to_file("/storage/heartbeat.bin", (uint8_t*)&myData.heartBeat, 1);
    }
    if(local_operation != LISTENING){
        std::vector<uint8_t> read_data = read_uint8_array_from_file("/storage/target.bin");
        if (read_data.size() == 6) {       
            memcpy(&broadcastAddress, read_data.data(), 6);
            ESP_LOGI(TAG, "Target MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
                    read_data[0], read_data[1], read_data[2],
                    read_data[3], read_data[4], read_data[5]);
        }
        else{
            broadcastAddress[0] = 0xFF; broadcastAddress[1] = 0xFF; broadcastAddress[2] = 0xFF;
            broadcastAddress[3] = 0xFF; broadcastAddress[4] = 0xFF; broadcastAddress[5] = 0xFF;
            ESP_LOGE(TAG, "Failed to read target MAC from file");
        }
        
        if(memcmp(broadcastAddress, general_broadcast_mac, 6) == 0){
            local_operation = LISTENING;
            write_uint8_array_to_file("/storage/opMode.bin", (uint8_t*)&local_operation, 1);
        }
    }

    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer");
        return;
    }

    while(1) {
        esp_err_t send_result;
        switch(local_operation){
            case LISTENING:
            case PAIRING:
                myData.operation = local_operation;
                send_result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            case NORMAL:
                vTaskDelay(pdMS_TO_TICKS(150));
                    // Set values to send

                myData.operation = NORMAL;
                ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &adc_raw[0][0]));
                ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_0, adc_raw[0][0]);
                myData.adc_value = adc_raw[0][0];
                myData.heartBeat++;
                
                // Send message via ESP-NOW
                send_result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
            
                if (send_result == ESP_OK) {
                    ESP_LOGI(TAG, "Sent with success");
                }
                else {
                    ESP_LOGI(TAG, "Error sending message %d", (int) send_result);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                vTaskDelay(pdMS_TO_TICKS(3000));
            break;
            case NODATA:
                break;
            case ERROR:
                // Error state
            break;
        }
    }
}