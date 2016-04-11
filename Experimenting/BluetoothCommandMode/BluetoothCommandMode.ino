/* File:      BluetoothCommandMode
 * Author:    Aaron Conrad
 * Modified:  4/6/16
 * 
 * Puts the HC-05 Bluetooth module into command mode
 * to change baud rate, name, and PIN. Uses code from
 * this website:
 * http://www.instructables.com/id/Modify-The-HC-05-Bluetooth-Module-Defaults-Using-A/step2/The-Arduino-Code-for-HC-05-Command-Mode/
 */

#include <SoftwareSerial.h>

const int TEENSY_RX = 0;
const int TEENSY_TX = 1;
const int COMMAND_PIN = 20;

SoftwareSerial BTSerial(TEENSY_RX, TEENSY_TX);

void setup() {
  pinMode(COMMAND_PIN, OUTPUT);
  digitalWrite(COMMAND_PIN, HIGH);
  delay(2000);
  digitalWrite(COMMAND_PIN, LOW);
  delay(2000);
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  BTSerial.begin(38400);
  delay(3000);

  //if (BTSerial.available()) {
    //Serial.write(BTSerial.read());
    //BTSerial.print("AT\r\n");
  //}
  Serial.println(BTSerial.read());
}

void loop() {
    /*if (BTSerial.available()) {
      Serial.println(BTSerial.read());
    }*/

  if (BTSerial.available()) {
    //Serial.write(BTSerial.read());
    BTSerial.print("AT+UART=115200,1,0\r\n");
  }
  if (Serial.available()) {
    BTSerial.write(Serial.read());
  }
}
