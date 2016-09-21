#include <PID_v1.h>

#define POT_PIN A1
#define MODE_PIN 1
#define PHASE_PIN 2
#define ENABLE_PIN 3

double target, input, output, speed;
uint16_t potInput;
double Kp = 1, Ki = 0, Kd = 0.05;
PID myPID(&input, &output, &target, Kp, Ki, Kd, DIRECT);

long angle = 0;
unsigned long startTime;
int iterations = 0;

void setup() {
  Serial.begin(9600);
  pinMode(MODE_PIN, OUTPUT);
  pinMode(PHASE_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(MODE_PIN, HIGH);

  potInput = analogRead(POT_PIN);
  input = map(potInput, 0, 1023, -130, 130);
  target = 511;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  startTime = millis();
}

void loop() {
  readSerial();
  
  potInput = analogRead(POT_PIN);
  input = map(potInput, 0, 1023, -130, 130);
  target = constrain(angle, -120, 120);
  myPID.Compute();

  
  //If negative, flip direction
  if (output < 0) {
    digitalWrite(PHASE_PIN, LOW);
  }
  else {
    digitalWrite(PHASE_PIN, HIGH);
  }
  speed = abs(output);

  //If the speed is too slow to turn motor, increase speed
  if (speed > 1 && speed < 25) {
    speed = 25;
  }
  else if (speed <= 1) {
    speed = 0;
  }
  analogWrite(ENABLE_PIN, speed);

  /*iterations++;
  if (millis() - startTime >= 10000) {
    Serial.print("Iterations: ");
    Serial.println(iterations);
    digitalWrite(ENABLE_PIN, 0);
    while(true){}
  }*/
  
  printDebug();
  //delay(100);
}

void printDebug() {
  Serial.print("Actual: ");
  Serial.print(input);
  Serial.print("  Target: ");
  Serial.print(target);
  Serial.print("  Output: ");
  Serial.println(output);
}

void readSerial() {
  char data[8];
  int dataIndex = 0;
  for (int i = 0; i < 8; i++) {
    data[i] = 0;
  }
  while (Serial.available() > 0) {
    //delay(3);
    if (Serial.available() > 0) {
      char c = Serial.read();
      //Serial.print("Next char: ");
      //Serial.println(c);
      data[dataIndex] = c;
      dataIndex++;
    }
    angle = atol(data);
    Serial.print("NEW TARGET: ");
    Serial.println(angle);
  }
}

