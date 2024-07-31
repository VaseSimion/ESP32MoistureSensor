#include <Arduino.h>
#include <U8g2lib.h>

void printMacAddress(Stream &Serial, uint8_t *sender_address, const char *title="MAC Address: ");
void displayBigText(U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C* display, const char* text);
