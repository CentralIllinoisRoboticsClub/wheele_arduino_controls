# Wheele Arduino Controls #

## Description ##

### wheele_ard_top ###
Collects input from multiple sensors and broadcasts on CAN bus. Samples 3D angular velocity, 1D absolute orientation (x-axis), 3D magnetometer, and 3D gravity vector, all from absolute orientation sensor (BNO055). Speed, steering, and dead-man switch pulse code modulation input used for RC/autonomous control. 

| Sensor/Input 	| Sample Rate	|
| ------------- |:-------------:|
| Gyroscope     | 50ms	        |
| Encoders      | 100ms		|
| Orientation   | 100ms		|
| Magnetometer	| 100ms		|
| Gravity	| 100ms		|
| Dead-Man PCM	| 100ms	  	|
| Steering PCM	| 100ms		|
| Speed PCM	| 100ms		|

### wheele_ard_bottom ###
Collects input from multiple sensors and broadcasts on CAN bus. Controls front/rear right/left steering servos based on commands received via CAN. Accepts both RC commands and autonomous CmdVel messages from ROS. Implements dead-man switch if no velocity/steering commands are received for more than 1 second. Samples battery voltage and current. Monitors position using interrupt-based right and left directional encoders. Simple digital input used for bump switch detection.

| Sensor/Input 	| Update Rate	|
| ------------- |:-------------:|
| Encoders      | 50ms		|
| Bump Switch	| 100ms		|
| Battery	| 1000ms	|

### wheele_ard_main ###
Collects input from multiple sensors and broadcasts on CAN bus. Monitors position using interrupt-based right and left directional encoders. Samples 3D angular velocity, 1D absolute orientation (x-axis), 3D magnetometer, and 3D gravity vector, all from absolute orientation sensor (BNO055). Speed, steering, and dead-man switch pulse code modulation inputs used for RC/autonomous control. Simple digital input used for bump switch detection. Battery voltage available to be sampled on analog input.


| Sensor/Input 	| Sample Rate	|
| ------------- |:-------------:|
| Gyroscope     | 50ms	        |
| Encoders      | 50ms		|
| Orientation   | 100ms		|
| Magnetometer	| 100ms		|
| Gravity	| 100ms		|
| Dead-Man PCM	| 100ms	  	|
| Bump Switch	| 100ms		|
| Steering PCM	| 200ms		|
| Speed PCM	| 200ms		|
| Battery	| 5000ms	|

### wheele_setup_ard ###
TBD

## Hardware ##
Main, top, and bottom boards are all Arduino Nano [TODO: Vendor/Processor?]


## Required Libraries ##
- Adafruit Unified Sensor
- Adafruit BNO055 (Absolute Orientation Sensor)
- SparkFun_CAN-Bus_Arduino_Library
- PinChangeInt
- EnableInterrupt
- CAN-BUS Shield v1.0.0
- Servo

## Building ##
Copy make-git-version to a location in the system PATH. For example, /usr/local/bin on Linux. Copy platform.local.txt to the hardware directoy of the selected build target. For example, .../hardware/arduino/avr/ if building for the AVR Arduino Nano build target. More explanation here: https://arduino.stackexchange.com/questions/23743/include-git-tag-or-svn-revision-in-arduino-sketch. Description of Arduino platform specification here: https://arduino.github.io/arduino-cli/0.19/platform-specification
