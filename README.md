# PinnenPilotESP
This project aims to build a simple autopilot, which is able to maintain the magnetic heading. It uses an MPU9250 sensor, a geared DC motor as an actuator and an and an ESP32 development board as a processor with wifi interface. The motor drives a shaft, which moves the tiller by a cord. The ESP generates a wifi AP with webserver, which enables to control the autopilot with a mobile device. The ESP also calculates a target yaw rate from the difference between actual heading and target heading and gives commands to the motor via a L298 H-bridge module. A small OLED is integrated to show basic information. The code is transferred to the esp via USB and Arduino IDE.
Most of the mechanical parts are 3d-printed. The stl- files are included.<br/>
Items needed:<br/>
DC geared motor 37 mm 12 V, 66 RPM<br/>
L298 H-bridge<br/>
0.91 Inch 128X32 IIC I2C White OLED LCD Display<br/>
ESP32 dev board<br/>
2 ball bearings 22mm (from inline skaters)<br/>
wires<br/><br/>
appr. 1.5m of cord<br/>

![IMG_20250926_113342](https://github.com/user-attachments/assets/10f7dbf1-3afe-4346-9344-481ae8cce848)
![IMG_20250926_113422](https://github.com/user-attachments/assets/660ce9b7-7906-4cd9-9fa0-f0f0dd629042)
![Screenshot_2025-09-26-14-32-38-889_org videolan vlc](https://github.com/user-attachments/assets/517e3b20-d279-4738-bdfa-d34ef4084f68)
