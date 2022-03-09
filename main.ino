#include <Arduino.h>
#include "./includes/oled.h"

extern Adafruit_SH1106G display;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //testdrawcircle();
  display.begin(i2c_Address,true);
  testdrawbitmap(logo16_glcd_bmp, LOGO16_GLCD_HEIGHT, LOGO16_GLCD_WIDTH);
}

void loop() {
  // put your main code here, to run repeatedly:
  //testdrawcircle();
  // Serial.println("hello");
  // delay(1000);
  // Serial.println(getCpuFrequencyMhz());
  // delay(1000);
}