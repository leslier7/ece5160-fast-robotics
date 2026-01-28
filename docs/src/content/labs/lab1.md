---
title: "Lab 1: Artemis Setup"
date: 2026-01-20
description: "Setting up the Artemis Nano board and running basic examples."
cover: "../../assets/lab1-cover.webp"
---

### Objective

The objective of this lab was to learn how to work with the Artemis Redboard Nano microcontroller board. To do this, the lab was divided into two parts, Lab 1A covered setting up the Arduino IDE and basic microcontroller operation. Lab 1B was about how to connect the Artemis to a computer over Bluetooth, and use Python and Jupyter Notebook to send commands to the Artemis.

### Lab 1A

#### Prelab

- The prelab for Lab 1A was to install the Arduino IDE and configure it to work with the Artemis.
- After installing the Arduino IDE, I setup and installed the SparkFun Apollo3 board manager

#### In Lab

- With the board manager installed, I could connect the Artemis to my computer via USB
  - I ran into problems with this step. I was running an older version of Linux Mint (21) on my laptop, and it wouldn't detect the Artemis as a USB device.
  - I tried to fix this by updating the CH340 driver, but even after installing it the Artemis didn't show up.
  - To temporarily fix this issue, I switched to the Windows 11 install on my laptop where the Artemis was detected.
  - I later tested the Artemis on my desktop running Mint 22.2, and it showed up as a USB device. I later updated my laptop's Mint install to 22, which fixed the USB problems. 
- With the Artemis now connected properly via USB, I was ready to flash the board with the *Blink* example to ensure that the board could be programmed. I confirmed that after flashing the board, the onboard LED did blink as expected.
- Next, I ran the *Serial* example to verify that I could send and recieve data from the Artemis over serial. The output is shown in Figure 1.

<figure>
  <img src="/ece5160-fast-robotics/assets/lab1/serial_output.png" alt="">
  <figcaption>Figure 1: Serial output from the Artemis</figcaption>
</figure>

- After verifying the serial communication, ran the *analogRead* example to test the temperature sensor. I looked at the serial output of the data, and then held my hand on the chip to verify that the temperature would increase due to my body heat. The output of the temperature sensor is shown in Figure 2.

<figure>
  <img src="/ece5160-fast-robotics/assets/lab1/temp_analog_read.png" alt="">
  <figcaption>Figure 2: Temperature Sensor Output</figcaption>
</figure>


