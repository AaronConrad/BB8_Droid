#include <PID_v1.h>

#define POT_PIN A1
#define LED_PIN 13

//Define Variables we'll be connecting to 
double Setpoint, Input, Output; 
 
//Specify the links and initial tuning parameters 
double Kp=2, Ki=5, Kd=1; 
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int iterations = 0;
float startTime;

void setup() {
  Serial.begin(9600);
  pinMode(LED_PIN, OUTPUT);
  Input = analogRead(POT_PIN);
  Setpoint = 100;
  myPID.SetMode(AUTOMATIC);
  startTime = millis();
}

void loop() {
  iterations++;
  Input = analogRead(POT_PIN);
  myPID.Compute();
  digitalWrite(LED_PIN, Output);
  //Serial.print("Input: ");
  //Serial.print(Input);
  //Serial.print("  Output: ");
  //Serial.println(Output);
  if (millis() - startTime > 10000) {
    Serial.print("Iterations: ");
    Serial.println(iterations);
    while (true) {}
  }
}
