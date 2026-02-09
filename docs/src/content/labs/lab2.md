---
title: "Lab 2: IMU"
date: 2026-02-04
description: "Setting up the IMU, filtering it, and sending data via Bluetooth test."
cover: "../../assets/lab2-cover.jpg"
---

## Objective

The objective of this lab was to learn how to work with the Inertial Measurement Unit (IMU). We are using the ICM-20948 over I²C, which allows us to use the Qwiic connector on the Artemis to communicate with the IMU using only one wire. We were also given our cars this week, and had to record a stunt with them to learn how the car works, mostly with the battery. 

## Lab Work

### IMU Library
To start the lab, we had to install the IMU library on in the Arduino IDE. I strongly dislike the Arduino IDE, so I decided to move to PlatformIO going forwards with my labs. PlatformIO is a program used for embedded development. It supports many different boards and frameworks, including Arduino. ne of the really nice features of PlatformIO is the `platformio.ini` file. Using this file, I can specify the board, framework, and libraries I am using. And if anyone else opens the project with PlatformIO, it will automatically install the board, framework, and libraries (even the version can be locked down). This means that I dont need to mess with the Arduino Board Manger or Library Manager anymore.

<details style="background: #f5f5f5; border: 1px solid #ddd; border-radius: 4px; padding: 0.5rem; margin: 1rem 0;">
<summary>platformio.ini file (click to expand)</summary>

```ini
[env:regular]
platform = https://github.com/nigelb/platform-apollo3blue.git
board = SparkFun_RedBoard_Artemis_Nano
framework = arduino
platform_packages = 
    framework-arduinoapollo3 @ https://github.com/sparkfun/Arduino_Apollo3.git
lib_deps = 
	sparkfun/SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library@^1.3.2
	arduino-libraries/ArduinoBLE@^1.5.0
monitor_speed = 115200
```
</details>

Although I used PlatformIO for the rest of the lab, I did use the Arduino IDE to run the example sketch for the IMU. This was done to make sure my IMU was working, and to play with the `AD0_VAL` pin. This pin sets the address on the I²C bus for the IMU. The default value is 1, and if changed to 0 the Artemis wont find the IMU. The user could bridge two pads on the back of the IMU board if the wanted to change the address. The two I²C addresses are 0x68 and 0x69 in hex. 

After verifying that the IMU could connect, I started playing with the IMU. I printed all the accelerometer, gyroscope, and magnetometer data over serial, and used SerialMonitor to plot it. An example output is shown in Figure 1. The data is clearly quite noisy. I did notice that one of the values on the accelerometer was always around 10,000 mg's. This value would shift depending on which orientation I held the IMU at (at right angles). This makes sense, since it is gravity pulling on the IMU. The gyroscope would react to movement, but would settle down to basically 0 if I didn't touch it. I didn't spend as much time looking at the magnetometer's data, since we won't be using it much, and it is harder to interpret. Finally, I added a 3 blink and a serial print to the board's startup, so I know the board is working properly.

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab2/Lab2SnapshotAll.svg" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 1: All IMU data</figcaption>
</figure>

### Accelerometer
The next part of the lab was focused on the accelerometer. I first had to calculate the pitch and roll of the IMU using the accelerometer. 