# servo
This repository contains the source code for 16 servos control using Adafruit's 16-Channel Servo shield. Because one PWM channel is damaged, this repo also demonstrates the use of built-in Servo library with digital pin. 

# list of materials
- 16 servos
- 1 Adafruit 16-Channel Servo/PWM shield
- 1 Arduino Uno
- 5V power supply

# set up 
1. Attach servo shield onto Arduino Uno.
2. Connect 16 servos to channels (0-15) on shield, except for the seventh. Connect vcc and gnd lines to power headers and the pwm pin of servo #7 to digital pin 7.
3. Connect power supply lines to both the shield and power plug of Arduino.

### potential issues
- Servo motors might not be perfectly aligned. To calibrate, set pwm of all servo motors to the same angle. Without rotating the motor itself, unscrew the servo arm and adjust angle.
