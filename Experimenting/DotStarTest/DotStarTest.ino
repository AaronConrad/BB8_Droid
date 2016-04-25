/* File:      DotStarTest
   Author:    Aaron Conrad
   Modified:  4/22/16

   Tests the brightness of DotStar LEDs with the color stated in the code.
*/

#include <Adafruit_DotStar.h>
#include <SPI.h>

#define STRAND_LENGTH 1
Adafruit_DotStar strip = Adafruit_DotStar(STRAND_LENGTH, DOTSTAR_BGR);
uint32_t color = 0xFFFFFF;
uint8_t brightness = 1;
boolean isGettingBrighter = true;

void setup() {
  strip.begin();
  strip.show();
  strip.setPixelColor(0, color);
  strip.show();
}

void loop() {
  if (brightness == 0) {
    isGettingBrighter = true;
  }
  else if (brightness == 255) {
    isGettingBrighter = false;
  }
  if (isGettingBrighter) {
    brightness++;
  }
  else {
    brightness--;
  }
  strip.setBrightness(brightness);
  strip.show();
  delay(10);
}
