/* File:      BluetoothBlink
   Author:    Aaron Conrad
   Modified:  3/30/16

   Simple program to test RFCOMM connection to other Bluetooth
   devices. Recieves number of times to blink the onboard LED,
   sends string back to sender confirming value, then blinks LED.

   Code and exercise is based on: http://www.uugear.com/portfolio/bluetooth-communication-between-raspberry-pi-and-arduino/
*/

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
