#include <SoftwareSerial.h>
SoftwareSerial bluetooth(0, 1);
int LED_PIN = 13;

void setup() {
  bluetooth.begin(9600);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  if (bluetooth.available() > 0) {
    int count = bluetooth.parseInt();
    if (count > 0) {
      bluetooth.print("You have input: ");
      bluetooth.println(String(count));
      blinkLED(count);
    }
  }
}

void blinkLED(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}

