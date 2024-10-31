
# Micro Arm
Project of robotic arm based on micro servomechanisms. 

It consists of 5 micro servos - 4 to control movement, 1 to control gripper. They are controlled by ESP32. The project utilizes additional PCA9685 board to control servos, potentiometers for manual control, CD4051 multiplexer to read from them and EEPROM memory for setting automatic path for robot. For peripherals, 2 separate I2C buses are used.

The project constist of 2 separate environments for testing and for executing. For testing, a Unity framework was chosen. The main application runs utilizing FreeRTOS.

## Features

- Home mode - reset arm position to default
- Manual mode - potentiometers control
- Automatic mode - reading path from EEPROM
- Gripper and mode control with tact switches
- Ability to modify path in EEPROM
- Logging to serial port


## Roadmap

- Add diodes indicating current mode - done 31.10.2024
- Add mode for teaching new path
- Add LCD display
- Create custom PCB
- Create 3D printed case for control panel

