/*  UltrasonicBlinking
    Aaron Conrad - 3/16/16

    This sketch reads the digital output of an ultrasonic sensor, and
    blinks the LED on the Teensy faster as an object gets closer to it.
*/

const int LED_PIN = 13;
const int PING_PIN = 23;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  delay(1000);
  pinMode(PING_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  long duration;
  double inches;
  long blink;
  duration = pulseIn(PING_PIN, HIGH);
  inches = duration / 147;
  Serial.print(inches);
  Serial.print("in, blink:");

  blink = 45 * (inches / 12);
  Serial.println(blink);
  digitalWrite(LED_PIN, HIGH);
  delay(blink);
  digitalWrite(LED_PIN, LOW);
  delay(1000 - blink);
}
