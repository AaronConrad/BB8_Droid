#include <Adafruit_DotStar.h>

#define STRAND_LENGTH 8
Adafruit_DotStar strip = Adafruit_DotStar(STRAND_LENGTH, 11, 14, DOTSTAR_BGR);
byte brightness = 0;
boolean isGettingBrighter = true;

void setup() {
  strip.begin();
  strip.show();

  strip.setPixelColor(0, 0x0000FF);
  strip.setPixelColor(1, 0x4040FF);
  strip.setPixelColor(2, 0xFF0000);
  strip.setPixelColor(3, 0xFF0000);
  strip.setPixelColor(4, 0x0000FF);
  strip.setPixelColor(5, 0xFFFFFF);
  strip.setPixelColor(6, 0xF0F0F0);
  strip.setPixelColor(7, 0x00FF00);
  strip.setBrightness(brightness);
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
