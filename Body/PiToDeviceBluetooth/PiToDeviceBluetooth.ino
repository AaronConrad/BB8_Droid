/* File:      PiToDeviceBluetooth
   Author:    Aaron Conrad
   Modified:  4/21/16

   In this sketch, the Teensy LC functions as a bridge between the 
   Raspberry Pi and a HC-06 Bluetooth module, though the Pi can be
   replaced by any USB Serial enbaled device.

   The reason I need this bridge is because I couldn't get the Pi
   and smart device to communicate through a RFCOMM Bluetooth Port.
   This solution is not as elegant and is slower, but is much easier
   to operate.

   Wanted but didn't work:  RasPi <--Bluetooth--> Smart device
   Solution:  RasPi <--USB Serial--> Teensy LC <--Bluetooth--> Smart device

   With this implementation, the data that comes from Bluetooth is NOT
   immediately sent over to the Pi. Instead, only when the Pi sends
   information to the device will the buffered data coming from the
   device be sent to the Pi.

   Later on, more specifics about amount of data and waiting for data to
   be sent instead of buffered will be added.
*/

//#include <SoftwareSerial.h>

const byte TEENSY_RX = 0;
const byte TEENSY_TX = 1;
//SoftwareSerial bluet(TEENSY_RX, TEENSY_TX);
#define bluet Serial1

byte dataToPi[32];
byte dataToPiIndex = 0;
byte dataFromPi[32];
byte dataFromPiIndex = 0;

void setup() {
  Serial.begin(115200);
  bluet.begin(115200);
}

void loop() {
}

void serialEvent() {
  collectPi();
  sendToBluetooth();
  collectBluetooth();
  sendToPi();
}

void collectBluetooth() {
  while (bluet.available()) {
    dataToPi[dataToPiIndex] = bluet.read();
    dataToPiIndex++;
  }
}

void collectPi() {
  while (Serial.available()) {
    dataFromPi[dataFromPiIndex] = Serial.read();
    dataFromPiIndex++;
  }
}

void sendToPi() {
  for (byte i = 0; i < dataToPiIndex; i++) {
    Serial.write(dataToPi[i]);
  }
  Serial.print('\n');
  dataToPiIndex = 0;
}

void sendToBluetooth() {
  for (byte i = 0; i < dataFromPiIndex; i++) {
    bluet.write(dataFromPi[i]);
  }
  bluet.print('\n');
  dataFromPiIndex = 0;
}

