# BB8_Droid
Life-size, functional replica of Star Wars' BB-8 with a Raspberry Pi,
Teensy LC, and various electronic devices. More information on the
components, goals, and features will be added in the coming weeks.

ELECTRONIC COMPONENTS:

(Main Body)
- Raspberry Pi 2 Model B
- Adafruit Motor HAT
- 2 brushed DC motors (forward motion)
- Servo(s) for side-to-side motion (undecided)
- Light continuous rotation servo (head rotation)
- 2 heavier continuous rotation servos (head position)
- Adafruit BNO055 accelerometer
- MPU-6050 acceleromter
- RGB LEDs
- Basic speaker with headphone jack
- Bluetooth Dongle

(Head)
- Teensy 3.2
- HC-05 Bluetooth module
- OLED Display
- RGB LEDs
- Maxbotix EV0 ultrasonic sensor

GOALS:

(Mechanically)
- Move forwards and backwards
- Lean side to side
- Rotate head 360 degrees
- Move head across the top of the ball
- Auto-stabilize
- Potentially add in-place rotation

(Software)
- Auto-stabilzation
- Fully controllable from phone/laptop via Bluetooth
- Main program (on Pi) built in Python since most of my libraries are in Python
- Take advantage of the Pi 2's quad-core processor with multithreading
- Make BB-8 as much a character as possible. This includes different states, such as:
  - Idle, doing nothing
  - Still, only head moving
  - Wander
  - Follow Me
  - User control

CHALLENGES:

- Interpret heading, roll, and pitch into something usable
- Auto-stabilizing with a robot with 4 degrees of freedom
- Splitting the tasks to be able to multithread safely
- Getting a smart device (phone), Pi, and Arduino all communicating
