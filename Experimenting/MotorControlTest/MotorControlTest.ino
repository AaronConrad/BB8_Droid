#define MODE_PIN 1
#define PHASE_PIN 2
#define ENABLE_PIN 3

bool dir = true;
bool accel = true;
uint8_t speed = 1;

void setup() {
  Serial.begin(9600);
  pinMode(MODE_PIN, OUTPUT);
  pinMode(PHASE_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(MODE_PIN, HIGH);
}

void loop() {
  if (speed >= 255) {
    accel = false;
    speed = 255;
  }
  else if (speed <= 0) {
    accel = true;
    dir = !dir;
    speed = 0;
  }

  if (accel) {
    speed++;
  }
  else {
    speed--;
  }

  Serial.print("Speed: ");
  Serial.println(speed);
  digitalWrite(PHASE_PIN, dir);
  analogWrite(ENABLE_PIN, speed);
  delay(50);
}
