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