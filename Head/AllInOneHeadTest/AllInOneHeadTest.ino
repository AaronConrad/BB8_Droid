//OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <SD.h>
#include <SPI.h>
#include <stdlib.h>

//LED
#include <Adafruit_DotStar.h>

//Accelerometer
#include "I2Cdev.h"

#include "MPU9150_9Axis_MotionApps41.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 mpu;
#define INTERRUPT_PIN 17

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

// If we are using the hardware SPI interface, these are the pins (for future ref)
#define sclk 14
#define mosi 11
#define ocs  9
#define rst  6
#define dc   5

// Color definitions
#define  BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0  
#define WHITE           0xFFFF

// to draw images from the SD card, we will share the hardware SPI interface
Adafruit_SSD1351 tft = Adafruit_SSD1351(ocs, dc, rst);

// For Arduino Uno/Duemilanove, etc
//  connect the SD card with MOSI going to pin 11, MISO going to pin 12 and SCK going to pin 13 (standard)
//  Then pin 10 goes to CS (or whatever you have set up)
#define SD_CS 10    // Set the chip select line to whatever you use (10 doesnt conflict with the library)

//ULTRASONIC SENSOR
#define ULTRASONIC 23
byte distance;

//LED STRANDS
#define LED1DATA 3
#define LED1CLOCK 4
#define LED1LENGTH 1
Adafruit_DotStar strip1 = Adafruit_DotStar(LED1LENGTH, LED1DATA, LED1CLOCK, DOTSTAR_BGR);

#define LED2DATA 15
#define LED2CLOCK 16
#define LED2LENGTH 1
Adafruit_DotStar strip2 = Adafruit_DotStar(LED2LENGTH, LED2DATA, LED2CLOCK, DOTSTAR_BGR);

//BLUETOOTH
#define bluet Serial1
byte bluetData = 0;

void setup() {
  //SETUP BLUETOOTH
  bluet.begin(9600);
  
  //SETUP THE OLED
  SPI.setSCK(14);
  Serial.begin(9600);
  pinMode(ocs, OUTPUT);
  digitalWrite(ocs, HIGH);
  // initialize the OLED
  tft.begin();
  //Serial.println("init");
  tft.fillScreen(CYAN);
  delay(500);
  tft.fillScreen(BLACK);
  //Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    //Serial.println("failed!");
    return;
  }
  //Serial.println("SD OK!");

  //SETUP SENSOR
  pinMode(ULTRASONIC, INPUT);

  //SETUP LEDS
  strip1.begin();
  strip1.show();
  strip1.setBrightness(8);
  strip1.setPixelColor(0, 0xFFFFFF);

  strip2.begin();
  strip2.show();
  strip2.setBrightness(8);
  strip2.setPixelColor(0, 0xFFFFFF);

  //SETUP ACCELEROMETER
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
    //while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(93);
    mpu.setYGyroOffset(33);
    mpu.setZGyroOffset(-34);
    mpu.setXAccelOffset(-1004);
    mpu.setYAccelOffset(701);
    mpu.setZAccelOffset(734); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }
}

void loop() {
// if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    getAccelerometerData();
    readUltrasonic();
    readBluetooth();
    strip1.setBrightness(distance / 2);
    strip1.show();
    strip2.setBrightness(128 - distance / 2);
    strip2.show();

    //Display data
    tft.fillScreen(BLACK);
    tft.setCursor(0,0);
    tft.setTextColor(CYAN);
    char buff[8];
    tft.print(dtostrf(ypr[0], 5, 2, buff));
    tft.setCursor(0, 10);
    tft.print(dtostrf(ypr[1], 5, 2, buff));
    tft.setCursor(0, 20);
    tft.print(dtostrf(ypr[2], 5, 2, buff));
    tft.setCursor(0, 40);
    tft.setTextColor(YELLOW);
    tft.print(String(distance));
    tft.setCursor(0, 60);
    tft.setTextColor(GREEN);
    tft.print(String(bluetData));
    delay(500);
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


void readUltrasonic() {
  long duration = pulseIn(ULTRASONIC, HIGH);
  distance = duration / 147;
}

void readBluetooth() {
  bluetData = 255;
  bluetData = bluet.read();
  Serial.println(bluetData);
  if (bluetData == 255) {
    return;
  }
  bluetData -= 48;
  bluetData %= 6;
  switch (bluetData) {
    case 0:
      strip1.setPixelColor(0, 0xFF0000);
      strip2.setPixelColor(0, 0xF0F000);
      break;
    case 1:
      strip1.setPixelColor(0, 0xF0F000);
      strip2.setPixelColor(0, 0x00FF00);
      break;
    case 2:
      strip1.setPixelColor(0, 0x00FF00);
      strip2.setPixelColor(0, 0x00F0F0);
      break;
    case 3:
      strip1.setPixelColor(0, 0x00F0F0);
      strip2.setPixelColor(0, 0x0000FF);
      break;
    case 4:
      strip1.setPixelColor(0, 0x0000FF);
      strip2.setPixelColor(0, 0xF000F0);
      break;
    case 5:
      strip1.setPixelColor(0, 0xF000F0);
      strip2.setPixelColor(0, 0xFF0000);
      break;
  }
}

