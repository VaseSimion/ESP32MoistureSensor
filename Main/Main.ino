#include <Arduino.h>
#include <U8g2lib.h> // from https://github.com/olikraus/u8g2

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C

const int sensorInputPin = 33;
int adcValueRead = 0;

// End of constructor list


void setup(void) {
  u8g2.begin();
}

void loop(void) {
  adcValueRead = analogRead(sensorInputPin);

  u8g2.clearBuffer();					// clear the internal memory
  u8g2.setFont(u8g2_font_u8glib_4_tf );	// choose a suitable font
  u8g2.drawStr(40,17,"ADC Value");	// write something to the internal memory
  u8g2.setFont(u8g2_font_luRS14_tr);	// choose a suitable font
  u8g2.drawUTF8(32,32,u8x8_u16toa(adcValueRead,4));	// write something to the internal memory
  
  u8g2.sendBuffer();					// transfer internal memory to the display
  delay(1000);  
}

