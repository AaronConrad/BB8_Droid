/* File:      BluetoothCommandMode
   Author:    Aaron Conrad
   Modified:  4/6/16

   Puts the HC-05 Bluetooth module into command mode
   to change baud rate, name, and PIN. Uses code from
   this website:
   http://www.instructables.com/id/Modify-The-HC-05-Bluetooth-Module-Defaults-Using-A/step2/The-Arduino-Code-for-HC-05-Command-Mode/

   The button must be pressed at power on in order for this sketch to work.
   
   Useable commands for my HC-05, all appended by "\r\n":
   AT
   AT+VERSION
   AT+NAME="newname"
   AT+UART=115200,1,0  where 115200 can be replaced by slower baud rates
   AT+PSWD="4digitnumber"
*/

#include <SoftwareSerial.h>

const int TEENSY_RX = 0;
const int TEENSY_TX = 1;
const int COMMAND_PIN = 20;

SoftwareSerial BTSerial(TEENSY_RX, TEENSY_TX);

void setup() {
  delay(3000);
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  //BTSerial.begin(38400);
  BTSerial.begin(9600);
  delay(3000);

  //Send command
  BTSerial.print("AT\r\n");
}

void loop() {
  //Get the module's feedback
  Serial.write(BTSerial.read());
  delay(100);
}
