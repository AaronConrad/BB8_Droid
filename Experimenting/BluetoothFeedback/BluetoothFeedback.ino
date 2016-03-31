/* File:      BluetoothFeedback
 * Author:    Aaron Conrad
 * Modified:  3/30/16
 * 
 * Simple Bluetooth feedback loop to test reading and
 * writing to the Bluetooth serial connection.
 */

#include <SoftwareSerial.h>

#define TEENSY_RX 0
#define TEENSY_TX 1
#define LED_PIN 13

String input;
SoftwareSerial BTSer(TEENSY_RX, TEENSY_TX);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);
  while (!Serial);

  BTSer.begin(9600);
  BTSer.println("Start up Bluetooth");
}

void loop() {
  if (BTSer.available()) {
    Serial.write(BTSer.read());
  }
  if (Serial.available()) {
    BTSer.write(Serial.read());
  }

  /*if (BTSer.available()) {
    input = char(BTSer.read());
  }
  if (input == "1") {
    digitalWrite(LED_PIN, HIGH);
  }
  else {
    digitalWrite(LED_PIN, LOW);
  }*/
  delay(100);
}
