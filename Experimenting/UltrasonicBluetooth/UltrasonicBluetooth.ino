#include <SoftwareSerial.h>

#define TEENSY_RX 0
#define TEENSY_TX 1

const int LED_PIN = 13;
const int PING_PIN = 23;
String input;
SoftwareSerial BTSer(TEENSY_RX, TEENSY_TX);

void setup() {
  Serial.begin(38400);
  BTSer.begin(9600);
  BTSer.println("Start up Bluetooth");
  delay(1000);
  pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH);
  pinMode(PING_PIN, INPUT);
}

void loop() {
  long duration;
  double inches;
  long blink;
  duration = pulseIn(PING_PIN, HIGH);
  inches = duration / 147;
  Serial.print(inches);
  Serial.print("in, blink:");
  if (BTSer.available()) {
    BTSer.println(inches);
  }
  delay(50);
  /*blink = 45 * (inches / 12);
  Serial.println(blink);
  digitalWrite(LED_PIN, HIGH);
  delay(blink);
  digitalWrite(LED_PIN, LOW);
  delay(1000 - blink);*/
}
