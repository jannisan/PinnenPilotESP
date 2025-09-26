# PinnenPilotESP
An autopilot for small boats
This project aims to build a simple autopilot, which is able to maintain the magnetic heading. It uses an MPU9250 sensor, a geared DC motor as an actuator and an and an ESP32 development board as a processor with wifi interface. The motor drives a shaft, which moves the tiller by strings. The ESP generates a wifi AP with webserver, which enables to control the autopilot with a mobile device. The ESP also calculates a target yaw rate from the difference between actual heading and target heading and gives commands to the motor via a L298 H-bridge module. A small OLED is integrated to show basic information. 
Most of the mechanical parts are 3d-printed.
you need:
DC geared motor 37 mm 12 V, 66 RPM
L298 H-bridge
0.91 Inch 128X32 IIC I2C White OLED LCD Display
ESP32 dev board
2 ball bearings (from inline skaters)
wires
1.5m of string
