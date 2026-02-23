---
title: "Lab 3: ToF"
date: 2026-02-11
description: "Setting up the ToF sensors, and sending data via Bluetooth test."
cover: "../../assets/lab3-cover.jpg"
---

## Objective
The objective of this lab was to connect two VL53L1X time of flight sensors to the Artemis, and transmit their distances over Bluetooth. To do this, we had to solder wires to the VL53L1X sensors because the verions that were provided did not have Qwiic connectors on them. This gave me the opportunity to practice my soldering.

## Lab work

### Prelab
For the prelab, I had to think about the wiring of VL53L1X sensors. The VL53L1X has a quirk for its I²C address: it is hard coded on the sensor (`0x29`), but can be changed in software. This means that just connecting two sensors on the same I²C bus without changing one of the sensor's addresses will cause both sensors to fail. To solve this problem, I solderd a JST connector to the Xshut pin on one of the sensors, and a JST connector to the Artemis. This is pictured in Figure 1. I choose to use a JST connector because I wanted the ability to easily remove the sensor, since it was connected via Qwiic already. I happened to have a kit of JST connectors lying around from another project, as well as a crimper so I could make my own cable. 

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab3/ToFWire.jpg" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 1: Both VL53L1X Sensors connected via Qwiic and JST connector</figcaption>
</figure>

With both sensors wired up, I was ready to write the software to connect them. To initialize both sensors, I set the GPIO connected to Xshut low to turn that sensor off, and then set the other sensor's I²C address to `0x30`. I checked to make sure that it worked, and then turned the original sensor on. This meant that the sensors addresses ended up as `0x29` and `0x30`.

<details style="background: #f5f5f5; border: 1px solid #ddd; border-radius: 4px; padding: 0.5rem; margin: 1rem 0;">
<summary>Setting sensor I²C address (click to expand)</summary>

```cpp
pinMode(A0, OUTPUT); //Used to control xshut on the side ToF
digitalWrite(A0, LOW);  // Xshut low to set I2C address

sensor.setI2CAddress(0x30);

int tries = 0;
while((sensor.getI2CAddress() != 0x30) && (tries < SETUP_TRIES)){
    DEBUG_PRINTLN("Error setting I2C address");
    tries++;
}

if (tries >= SETUP_TRIES){
    return false;
}

digitalWrite(A0, HIGH);
```
</details>

With the sensors connected, I had to think about placement. I am going to place one sensor at the front of the car, and one on the right side. This will allow my car to follow walls, but only if the wall is on the right. I won't be able to see anything on the left side of the car.

# TODO wiring diagram

### Lab

#### Battery

As part of this lab, we had to power the Artemis with a battery. Part of this included soldering a JST connector on the battery. The default JST polarity was inverted, so I would have had to solder the red battery lead to the black JST lead. This bothers me at a fundamental level, so I used a pair of tweasers to pull the wires out of the JST connector, and swapped them so all the wire colors matched. Figure 2 shows the swapping technique, and Figure 3 shows the polarity going into the Artemis.

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab3/JSTSwap.jpg" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 2: JST wire swapping technique</figcaption>
</figure>

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab3/JSTSwap.jpg" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 3: JST polarity on Artemis</figcaption>
</figure>

#### ToF Software

With the soldering done, it was time to start the software portion of the lab. First, I ran the build in `Example05_wire_I2C` which scans the I²C bus and prints the addresses of anything connected. I started by plugging in one ToF sensor, and the output is shown in Figure 4. It showed `0x29` as expected. I then plugged both ToF sensors in without changing the code at all. This is shown in Figure 5, and shows no addresses on the bus. Finally, Figure 6 shows one ToF sensor and the IMU to show that both sensors work when connected.

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab3/Only1ToFScan.png" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 4: Only 1 ToF sensor on I²C bus</figcaption>
</figure>

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab3/Only1ToFScan.png" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 5: Both ToF sensors on I²C bus with no modifications</figcaption>
</figure>

<figure style="max-width: 500px;">
  <img src="/ece5160-fast-robotics/assets/lab3/ToFAndIMUScan.png" alt="" style="max-height: 400px; width: auto; object-fit: contain;">
  <figcaption>Figure 6: ToF and IMU on I²C bus</figcaption>
</figure>

# TODO sensor data and mode

# TODO how to show sensors working in parallel

### Additional Tasks (5000-Level)
# TODO color sensitivity and texture