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
The next part of the lab was focused on the accelerometer. I first had to calculate the pitch and roll of the IMU using the accelerometer. I used the `atan2()` function to do this so the output goes from -180 to 180 degrees (`atan()` only goes from -90 to 90). After calculating the pitch and roll, I printed them over serial and printed the output of moving it 90 degrees on each axis, shown in Figure 2. Looking at the figure, you can see the 90 degree rotations. But the axis which isn't moving is very noisy. I think this is from my hands shaking while I rotate the board, the vibrations calmed down if I held the IMU at the bottom of the board and braced it using my fingers resting on the table.

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab2/AccelBasicTest.png" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 2: Accelerometer 90 Degree Data</figcaption>
</figure>

#### FFT
After plotting the accelerometer data in the time domain, I also plotted the data in the frequency domain using an FFT. Figure 3 shows the FFT of Figure 2. As shown, most of the noise and accelerations are pretty low frequency, with a large frequency around 0. I think that this large spike is caused by me rotating the board. 

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab2/AccelBasicFFT.png" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 3: Accelerometer 90 Degree FFT</figcaption>
</figure>

With this data, I could figure out what cutoff frequency to use for a low pass filter, and the associcated alpha value. A low pass filter is used to filter out some of the noise and frequencies that I dont want the IMU to read. The alpha value is used to weight the low pass filter, and is determined by the cutoff frequency.

Low pass filter equation:

$$
\theta_{\mathrm{LPF}}[n] = \alpha\,\theta_{\mathrm{RAW}} + (1-\alpha)\,\theta_{\mathrm{LPF}}[n-1]
$$

$$
\theta_{\mathrm{LPF}}[n-1] = \theta_{\mathrm{LPF}}[n]
$$

Alpha calculations:

$$
RC = \frac{1}{2\pi \cdot fc} = \frac{1}{2\pi \cdot 15}
    = 0.0106
$$

$$
\alpha = \frac{T}{T + rc} = \frac{0.034}{0.034 + 0.0106}
       = 0.762
$$

Note that *T* is the sample time, so it depends on the execution time of the loop, so alpha could change depending on the program. I started with a cutoff frequency of 15 Hz, because I wanted to make sure that my filter was effective, but would sill be responsive.

I tested the filter, and compared the raw data to the filtered data, which can be seen in Figure 4. The LPF seems to be working, as the filtered response is generally much smoother than the raw data. There is still some bumps on the graph, but mostly when I am moving the IMU myself.

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab2/AccelTimeLPF.png" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 4: Accelerometer LPF Graph</figcaption>
</figure>

I also took the FFT of both the LPF and raw data to compare them, shown in Figure 5. There is still a large spike close to 0 Hz, but the filtered lines generally have less amplitude across the frequency spectrum.

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab2/AccelLPFFFT.png" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 5: Accelerometer FFT of LPF Graph</figcaption>
</figure>

### Gyroscope
With the accelerometer filtered, it was time to look at the gyroscope. The gyroscope measures rate of change, so in order to estimate the pitch and roll, we start with an initial value (assumed to be 0 in this case) and then integrate the values. 

$$
\begin{aligned}
\text{pitch}_{k+1} &= \text{pitch}_{k} + \omega_y \cdot \Delta t \\
\text{roll}_{k+1}  &= \text{roll}_{k}  + \omega_x \cdot \Delta t \\
\text{yaw}_{k+1}   &= \text{yaw}_{k}   + \omega_z \cdot \Delta t
\end{aligned}
$$

This has the advantage of being able to calculate the yaw, which isn't possible with the accelerometer with only one reference vector (gravity). The downside to this integration approach is that error grows over time, so the sensor will drit and become unreliable. The advantage is that the gyro isn't very noisy. I plotted the gyro angle estimations, as well as the FFT, shown in Figures 6 and 7.

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab2/GyroTimeData.png" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 6: Gyroscope Pitch, Roll, and Yaw Calculations</figcaption>
</figure>

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab2/GyroFFTData.png" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 7: Gyroscope FFT</figcaption>
</figure>

In Figure 7, we can see that other than the spikes around 0 Hz, the noise on the gyroscope is significantly lower than the accelerometer. I also tried removing the loop delay to increase the sampling rate, which is shown in Figure 8. I think that it makes it more sensitive, but also increases the error because the error is being added more frequently.

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab2/GyroNoDelayData.png" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 8: Gyroscope Data No Delay</figcaption>
</figure>

### Complementary Filter
With ways to get the pitch and roll from both the accelerometer and gyroscope, the question becomes "How can I take advantage of both sensors to use their strengths, and reduce their weaknesses?" The complementary filter is a way to combine the data from both sensors to make a more accurate prediction. The goal of a complementary filter is to trust the gyroscope over very short timescales because it has lower noise, but over longer timescales trust the accelerometer because it is more accurate. The equation for a complementary filter is as follows:

$$
\theta = (\theta + \theta_g)(1-\alpha) + \theta_a \alpha
$$

where $\theta_g$ is the angle from the gyroscope, $\theta_a$ is from the accelerometer, and $\alpha$ is used to weight both. A low theta will trust the gyro in the short term, and the accelerometer in the longer term, which is the behavior that we expect. After coding the complementary filter, I set it up so that it would print the data from the complementary filter, accelerometer, and gyroscope so that I could compare them. 

# TODO add code

## Stunts
As part of the lab, to familiarize ourselves with the RC car, we had to record a stunt. I filmed a pretty basic stunt, just going forwards and spinning out. This stunt is pretty basic, but part of that is because my apartment doesn't have a ton of room for complex stunts. (I didn't film in the lab because I kept getting interference from other students). I found that the car was somewhat hard to control, at least with the remote. It rotated very quickly, which made lining it up on my hallway was difficult. This did mean that it turned pretty quickly while driving. 

<figure style="max-width: 720px; margin: 1rem 0;">
  <iframe
    width="100%"
    height="405"
    src="https://www.youtube-nocookie.com/embed/uYCLfR_zXc0"
    title="Lab 2 stunt video"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
  <figcaption>Figure X: RC Car Stunt</figcaption>
</figure>

Another reason that I filmed such a basic stunt is because I was having troubles with the basic connection to my car. It would sometimes stop responding, or register inputs that I didn't do. I know that we weren't meant to film any aerial stunts, but this one happend totally by accident (I was just holding the forward button), so I'm going to share it anyways. (You can hear my frustration at the end when I am pressing buttons but the car isn't responding).

<figure style="max-width: 720px; margin: 1rem 0;">
  <iframe
    width="100%"
    height="405"
    src="https://www.youtube-nocookie.com/embed/s_CmSF7MAyc"
    title="Lab 2 stunt video"
    frameborder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
  </iframe>
  <figcaption>Figure X: RC Stunt Gone Wrong</figcaption>
</figure>