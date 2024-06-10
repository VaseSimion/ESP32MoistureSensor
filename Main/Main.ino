#include <ESP8266WiFi.h>
#include <espnow.h>
#include <U8g2lib.h> // from https://github.com/olikraus/u8g2
#include "user_interface.h"
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

//This reads the value and sends it through ESP now and it's done for ESP8266
//It gets values from ESP Now but it ignores them for the moment

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);

const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
int adcValueRead = 0;
int rssi = 0;

uint8_t broadcastAddress[] = {0xCC, 0x7B, 0x5C, 0x28, 0xD4, 0x50};
uint8_t ESP_OUI[] = {0x18, 0xFE, 0x34};
String success;

typedef enum {
      WIFI_PKT_MGMT,  /**< Management frame, indicates 'buf' argument is wifi_promiscuous_pkt_t */
      WIFI_PKT_CTRL,  /**< Control frame, indicates 'buf' argument is wifi_promiscuous_pkt_t */
      WIFI_PKT_DATA,  /**< Data frame, indiciates 'buf' argument is wifi_promiscuous_pkt_t */
      WIFI_PKT_MISC,  /**< Other type, such as MIMO etc. 'buf' argument is wifi_promiscuous_pkt_t but the payload is zero length. */
  } wifi_promiscuous_pkt_type_t;

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

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Heartbeat: ");
  Serial.println(receivedData.heartBeat);
  ogSender = receivedData.idOfOgSender;
  adcValue = receivedData.adcValue;
  heartBeat = receivedData.heartBeat;
}


void promiscuous_rx_cb(uint8_t *buf, uint16 len) { //This might not be accurate but it is the right size
typedef struct {
    signed rssi:8;                /**< Received Signal Strength Indicator(RSSI) of packet. unit: dBm */
    unsigned rate:5;              /**< PHY rate encoding of the packet. Only valid for non HT(11bg) packet */
    unsigned :1;                  /**< reserved */
    unsigned sig_mode:2;          /**< Protocol of the reveived packet, 0: non HT(11bg) packet; 1: HT(11n) packet; 3: VHT(11ac) packet */
    unsigned :16;                 /**< reserved */
    unsigned mcs:7;               /**< Modulation Coding Scheme. If is HT(11n) packet, shows the modulation, range from 0 to 76(MSC0 ~ MCS76) */
    unsigned cwb:1;               /**< Channel Bandwidth of the packet. 0: 20MHz; 1: 40MHz */
    unsigned :12;                 /**< reserved */
    unsigned smoothing:1;         /**< Set to 1 indicates that channel estimate smoothing is recommended.
                                       Set to 0 indicates that only per-carrierindependent (unsmoothed) channel estimate is recommended. */
    unsigned not_sounding:1;      /**< Set to 0 indicates that PPDU is a sounding PPDU. Set to 1indicates that the PPDU is not a sounding PPDU.
                                       sounding PPDU is used for channel estimation by the request receiver */
    unsigned :1;                  /**< reserved */
    unsigned aggregation:1;       /**< Aggregation. 0: MPDU packet; 1: AMPDU packet */
    unsigned stbc:2;              /**< Space Time Block Code(STBC). 0: non STBC packet; 1: STBC packet */
    unsigned fec_coding:1;        /**< Forward Error Correction(FEC). Flag is set for 11n packets which are LDPC */
    unsigned sgi:1;               /**< Short Guide Interval(SGI). 0: Long GI; 1: Short GI */
    unsigned noise_floor:8;   /**< noise floor */
    unsigned ampdu_cnt:8;     /**< ampdu cnt */
    unsigned channel:4;       /**< which channel this packet in */
    unsigned :12;             /**< reserve */
  } wifi_pkt_rx_ctrl_t;

typedef struct {
    wifi_pkt_rx_ctrl_t rx_ctrl; /**< metadata header */
    uint8_t payload[0];       /**< Data or management payload. Length of payload is described by rx_ctrl.sig_len. Type of content determined by packet type argument of callback. */
  } wifi_promiscuous_pkt_t;
  
  typedef struct {
     unsigned frame_ctrl: 16;
     unsigned duration_id:16; 
     uint8_t addr1[6]; /* receiver address */
     uint8_t addr2[6]; /* sender address */
     uint8_t addr3[6]; /* filtering address */
     unsigned sequence_ctrl:16;
     unsigned category:8;
     uint8_t addr4[6]; /* optional */
  } wifi_ieee80211_mac_hdr_t;

    typedef struct {
      wifi_ieee80211_mac_hdr_t hdr;
      uint8_t payload[0]; // network data ended with 4 bytes csum (CRC32) 
    } wifi_ieee80211_packet_t;


    static const uint8_t ACTION_SUBTYPE = 0xd0;
    const wifi_promiscuous_pkt_t *package = (wifi_promiscuous_pkt_t *)buf;

   const wifi_ieee80211_packet_t *pk = (wifi_ieee80211_packet_t*)package->payload;

   const wifi_ieee80211_mac_hdr_t *hdr = &pk->hdr;
   
   const uint8_t *data = pk->payload;

  // Only continue processing if this is an action frame containing the Espressif OUI.
    if ((ACTION_SUBTYPE == (hdr->frame_ctrl & 0xFF)) &&
        (memcmp(hdr->addr4, ESP_OUI, 3) == 0)) {
        rssi = package->rx_ctrl.rssi;
        heartBeat = data[11];
        ogSender = data[7];
        //for(int i=0;i<len;i++)
        //  Serial.print(buf[i],HEX);
        //Serial.println();
/*        Serial.println(data[0]);
        Serial.println(data[1]);
        Serial.println(data[2]);
        Serial.println(data[3]);
        Serial.println(data[4]);
        Serial.println(data[5]);
        Serial.println(data[6]);
        Serial.println(data[7]);
        Serial.println(data[8]);
        Serial.println(data[9]);
        Serial.println(data[10]);
        Serial.println(data[11]);
        Serial.println(data[12]);
        Serial.println(data[13]);
        Serial.println(data[14]);
        Serial.println(data[15]);
        Serial.println(data[16]);
        Serial.println(data[17]);
        Serial.println("");*/
     }
}

void setup(void) {
    // Init Serial Monitor
  Serial.begin(115200);

  u8g2.begin();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  wifi_promiscuous_enable(1);
  wifi_set_promiscuous_rx_cb(promiscuous_rx_cb);
//  wifi_promiscuous_set_mac(broadcastAddress);
  sendData.idOfOgSender = 1;
  sendData.heartBeat = 0;
}

void loop(void) {
  sendData.heartBeat++;
    delay(500);
    // Send message via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *) &sendData, sizeof(sendData));

  {
    u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFont(u8g2_font_u8glib_4_tf );	// choose a suitable font
    u8g2.drawStr(20,17,"ESP number");	// write something to the internal memory

    u8g2.drawUTF8(90,17,u8x8_u16toa(ogSender, 2));	// write something to the internal memory

    u8g2.drawUTF8(110,17,u8x8_u16toa(heartBeat,2));	// write something to the internal memory

    u8g2.setFont(u8g2_font_8x13B_tr);	// choose a suitable font
    //u8g2.drawUTF8(50,32,u8x8_u16toa(stored_receivedData[sensor_id].adcValue,4));	// write something to the internal memory

    char str_rssi[4];
    sprintf(str_rssi, "%d", rssi);
    u8g2.drawUTF8(50,32,str_rssi);	// write something to the internal memory
  
    u8g2.sendBuffer();					// transfer internal memory to the display
  }
  delay(1000);
}

