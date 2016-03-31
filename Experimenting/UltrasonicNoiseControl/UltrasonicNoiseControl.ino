/* File:      UltrasonicNoiseControl
   Author:    Aaron Conrad
   Modified:  3/30/16

   Runs ultrasonic sensor at maximum rate (20Hz), stores
   previous readings, and computes a running average that
   is then sent out to Serial and/or Bluetooth at 20Hz.

   The number of previous readings stored and number of outliers
   that can be thrown out is easily changeable, though more than
   ~3 outliers and 8 readings can lower max rate.
*/

#include <SoftwareSerial.h>

//Pinouts
const byte BOARD_LED = 13;
const byte ULTRASONIC = 23;
const byte BLUETOOTH_RX = 0;
const byte BLUETOOTH_TX = 1;

SoftwareSerial bluet(BLUETOOTH_RX, BLUETOOTH_TX);

//Sensor attributes
const byte STACK_SIZE = 4;
const byte OUTLIERS = 1;

byte distances[STACK_SIZE];
bool isOutlier[STACK_SIZE];
byte stackIndex = 0;
byte avgDist;

bool ledState = true;

unsigned long startTime;

void setup() {
  Serial.begin(38400);
  bluet.begin(9600);
  bluet.println("Start up Bluetooth");
  pinMode(BOARD_LED, OUTPUT);
  pinMode(ULTRASONIC, INPUT);
  delay(500);   //Allow sensor to boot up
}

void loop() {
  startTime = millis();

  //Reset outlier data
  for (byte i = 0; i < STACK_SIZE; i++) {
    isOutlier[i] = false;
  }

  //Get next reading and throw out oldest one
  distances[stackIndex] = readUltrasonic();
  stackIndex = (stackIndex + 1) % STACK_SIZE;

  avgDist = getAverageDist();
  for (byte i = 0; i < OUTLIERS; i++) {
    findOutlier();
  }
  avgDist = getAverageDist();

  printData();
  //printDebug();
  // delay(1000);
}

/*
   Gets a distance reading from the ultrasonic sensor
   and determines the distance in inches
*/
byte readUltrasonic() {
  long duration = pulseIn(ULTRASONIC, HIGH);
  return duration / 147;
}

/*
   Calculates the average distance, not including outliers
*/
byte getAverageDist() {
  int avg = 0;
  for (byte i = 0; i < STACK_SIZE; i++) {
    if (!isOutlier[i]) {
      avg += distances[i];
    }
  }
  return avg / (STACK_SIZE - OUTLIERS);
}

/*
   Finds one outlier from the set and marks it, ignoring
   any outliers already marked.
*/
void findOutlier() {
  int diffs[STACK_SIZE];    //May be negative
  for (byte i = 0; i < STACK_SIZE; i++) {
    //If already determined to be outlier, skip
    if (isOutlier[i]) {
      diffs[i] = 0;
    }
    else {
      diffs[i] = avgDist - distances[i];
      diffs[i] = abs(diffs[i]);
    }
  }

  byte outlier = diffs[0];
  byte outlierIndex = 0;
  for (byte i = 1; i < STACK_SIZE; i++) {
    if (diffs[i] > outlier) {
      outlier = diffs[i];
      outlierIndex = i;
    }
  }
  isOutlier[outlierIndex] = true;
}

void printData() {
  Serial.print(avgDist);
  Serial.print(" in, ");
  Serial.print(millis() - startTime);
  Serial.println(" ms");
  if (bluet.available()) {
    bluet.print(avgDist);
    bluet.print(" in, ");
    bluet.print(millis() - startTime);
    bluet.println(" ms");
  }
}

void printDebug() {
  Serial.print("[");
  for (byte i = 0; i < STACK_SIZE; i++) {
    Serial.print(distances[i]);
    if (isOutlier[i]) {
      Serial.print("*");
    }
    Serial.print(", ");
  }
  Serial.println("]");

  if (bluet.available()) {
    bluet.print("[");
    for (byte i = 0; i < STACK_SIZE; i++) {
      bluet.print(distances[i]);
      if (isOutlier[i]) {
        bluet.print("*");
      }
      bluet.print(", ");
    }
    bluet.println("]");
  }
}
