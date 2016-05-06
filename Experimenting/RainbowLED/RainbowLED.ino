/* File:      RainbowLED
   Author:    Aaron Conrad
   Modified:  4/22/16

   DotStar LED strand test that smoothly cycles through red to green to blue,
   i.e. the rainbow.
*/

#include <Adafruit_DotStar.h>
#include <SPI.h>

#define STRAND_LENGTH 1
Adafruit_DotStar strip = Adafruit_DotStar(STRAND_LENGTH, DOTSTAR_BGR);
uint8_t red = 0xFF;
bool isRedIncreasing = false;
uint8_t green = 0x00;
bool isGreenIncreasing = true;
uint8_t blue = 0x00;
bool isBlueIncreasing = true;
uint8_t brightness = 64;
uint32_t color;
long iteration = 0;

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.show();
  strip.setBrightness(brightness);
}

// Cycle through rainbow
void loop() {
  // Set direction of color change
  if (red == 0xFF) {
    isRedIncreasing = false;
    isGreenIncreasing = true;
    isBlueIncreasing = false;
  }
  else if (green == 0xFF) {
    isRedIncreasing = false;
    isGreenIncreasing = false;
    isBlueIncreasing = true;
  }
  else if (blue == 0xFF) {
    isRedIncreasing = true;
    isGreenIncreasing = false;
    isBlueIncreasing = false;
  }
  if (isRedIncreasing) {
    red = red + 1;
    blue = blue - 1;
  }
  else if (isGreenIncreasing) {
    green = green + 1;
    red = red - 1;
  }
  else if (isBlueIncreasing) {
    blue = blue + 1;
    green = green - 1;
  }
  color = (red * 0x10000) + (green * 0x100) + blue;
  //Serial.println(color, HEX);
  strip.setPixelColor(0, color);
  strip.setPixelColor(1, color);
  strip.show();
  //Serial.println(iteration);
  iteration++;
  delay(100);
}
