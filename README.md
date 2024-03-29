# Pill dispenser
![image](https://github.com/andreagy/pill-dispenser/assets/112083530/77c55d1f-cfc0-4844-9a72-17acd3afa944)![image](https://github.com/andreagy/pill-dispenser/assets/112083530/65f52f5a-0ccc-4e6e-a57b-10d34dcf63a3)

### Embedded Systems Programming course project at Metropolia University of Applied Sciences

### Main parts:
- controller PCB: Raspberry Pi Pico on a development board, using Pico C SDK
- dispenser: a wheel with 8 compartments, inside the base is a stepper motor and a piezo electric sensor for detecting falling pills
- optical sensor for calibrating
- EEPROM for storing system status

#### The project involves developing an automated pill dispenser with the following features:
The dispenser releases daily medication at a set time, using a piezo electric sensor to confirm if a pill was dispensed. One compartment out of the eight is for calibration. The system state, including the calibration and dispensing status is stored in non-volatile memory to persist across boots/power offs. The device detects if it was turned off while turning, and resets to the starting point without dispensing the remaining pills.
For testing purposes, the interval between dispensing is set to 30 seconds. The user can interact with the device through button presses.
