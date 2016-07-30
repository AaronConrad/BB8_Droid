//OLED Display
#include <stdlib.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
//LED Strands
#include <Adafruit_DotStar.h>
//Accelerometer/Gyroscope
#include <I2Cdev.h>
#include <MPU9150_9Axis_MotionApps41.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

//Pin Mappings
//Not all these are used in code, but are here for reference
#define BLUETOOTH_RX  0
#define BLUETOOTH_TX  1
#define STRAND1_DATA  2
#define STRAND1_CLK   3
#define OLED_DC       5
#define OLED_RST      6
#define OLED_OCS      9
#define OLED_SCS      10
#define OLED_MOSI     11
#define OLED_MISO     12
#define BOARD_LED     13
#define OLED_CLK      14
#define STRAND2_CLK   15
#define STRAND2_DATA  16
#define MPU_INTERRUPT 17
#define MPU_DATA      18
#define MPU_CLK       19
#define SONIC_DATA    23

//Use hardware SPI for OLED display
Adafruit_SSD1351 oled = Adafruit_SSD1351(OLED_OCS, OLED_DC, OLED_RST);
// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

//Use bit-bashed SPI for LED strands
#define STRAND1_LENGTH  4     //eye, mouth, and side panel
#define LED_MOUTH       1     //indices for these leds
#define LED_EYE         2
#define LED_SIDE_1      3
#define LED_SIDE_2      4
Adafruit_DotStar leds = Adafruit_DotStar(STRAND1_LENGTH, STRAND1_DATA, STRAND1_CLK, DOTSTAR_BGR);

#define STRAND2_LENGTH  3     //holoprojector
Adafruit_DotStar holo = Adafruit_DotStar(STRAND2_LENGTH, STRAND2_DATA, STRAND2_CLK, DOTSTAR_BGR);

//Use hardware serial for Bluetooth
#define BLUETOOTH_BUFFER_SIZE   8
#define bluet Serial1
byte bluetData[BLUETOOTH_BUFFER_SIZE];

//Ultrasonic sensor
byte distance;

//Use DMP for Accelerometer/Gyroscope/Magnetometer
//Code is from Jeff Rowberg
MPU9150 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int16_t mag[3];         // [x, y, z]            raw magnetic values

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

//Initialization
void setup() {
  //Setup Bluetooth
  bluet.begin(9600);

  //Setup LED strands
  leds.begin();
  leds.show();
  holo.begin();
  holo.show();

  //Setup ultrasonic sensor
  pinMode(SONIC_DATA, INPUT);

  //Setup OLED
  SPI.setSCK(OLED_CLK);   //Use alternate SPI clock pin
  pinMode(OLED_OCS, OUTPUT);
  digitalWrite(OLED_OCS, HIGH);
  oled.begin();
  oled.fillScreen(BLACK);
  if (!SD.begin(OLED_SCS)) {
    //SD card failed, don't continue
    showFatalError();
  }

  //Setup accelerometer/gyroscope/magnetometer
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();
  pinMode(MPU_INTERRUPT, INPUT);
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    //DMP programming faied, don't continue
    showFatalError();
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}

void getAccelerometerData() {
  // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        delay(20);
    }

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        ypr[0] = ypr[0] * 180 / M_PI;
        ypr[1] = ypr[1] * 180 / M_PI;
        ypr[2] = ypr[2] * 180 / M_PI;
    }
}

/*  Take a reading from the ultrasonic sensor. Uses the PWM
 *  output of the sensor.
 */
void getUltrasonicReading() {
  long duration = pulseIn(SONIC_DATA, HIGH);
  distance = duration / 147;
}

/*  A component doesn't work and would prevent proper operation
    of the head. Flash the leds and oled display to indicate a
    reboot is neccessary. There is no exit from this function
    aside from turning off the device.
*/
void showFatalError() {
  while (true) {
    oled.fillScreen(RED);
    delay(500);
    oled.fillScreen(BLACK);
    delay(500);
  }
}
