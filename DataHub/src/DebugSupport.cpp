#include "DebugSupport.h"

#define MAC_SIZE 6

void printMacAddress(Stream &Serial, uint8_t *sender_address, const char *title){
    Serial.print(title);
    for(int i = 0; i < MAC_SIZE; i++){
        Serial.printf("%02X",sender_address[i]);
        if(i < MAC_SIZE - 1){
            Serial.print(":");
        }
    }
    Serial.println();
}

void displayBigText(U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C* display, const char* text){
    display->clearBuffer();
    display->setFont(u8g2_font_8x13B_tr);
    display->drawStr(40,25,text);
    display->sendBuffer();
}