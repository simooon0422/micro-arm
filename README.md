
# Micro Arm
Project of robotic arm based on micro servomechanisms. 

It consists of 5 micro servos - 4 to control movement, 1 to control gripper. They are controlled by ESP32. The project utilizes additional PCA9685 board to control servos, potentiometers for manual control, CD4051 multiplexer to read from them and EEPROM memory for setting automatic path for robot. For peripherals, 2 separate I2C buses are used. Control panel utilizes LCD connected via SPI.

The project constist of 2 separate environments for testing and for executing. For testing, a Unity framework was chosen. The main application runs utilizing FreeRTOS.

## Features

- Home mode - reset robot position to default
- Manual mode - control robot with potentiometers
- Automatic mode - move in a loop according to sequence read from EEPROM
- Teach mode - modify path in EEPROM for automatic mode
- Diodes indicating current mode
- Control panel with LCD display, potentiometers and tact switches
- Logging to serial port
- Custom PCBs
- Custom 3D printed case
