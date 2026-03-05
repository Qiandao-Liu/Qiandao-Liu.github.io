---
layout: archive
title: "Lab 3: Time of Flight Sensors"
permalink: /mae4190/lab3/
author_profile: true
---

{% include base_path %}

## Prelab

The goal of this lab is to set up the time-of-flight sensors and get them ready to be mounted on the RC car in later labs. These sensors work by emitting infrared light and measuring the time it takes for the light to reflect back, which allows them to estimate the distance to nearby objects.

One of the sensors was placed at the front of the car. This placement was the most straightforward choice since the car will usually be driving forward.

For the second sensor. One idea was to place it near the front as well, but at a slightly different angle, which could provide some redundancy and a wider sensing area. I eventually mount the second sensor on right side of the car. Because in future lab we will do path planning and mapping, and side sensor can make sure car can go through the wall.

The final orientation and placement of the sensors are shown in the figure below.

<img src='/images/mae4190/lab3/lab3_fig2.png' width='700'>


To communicate to the two IMUs, either each one had to be communicated to one at a time or the address of one needed to be changed, as they both start with the same I2C address of 0x52. I decided to change the address of one on program startup, as the other approach means data can't be gotten from both sensors simultaneously. I decided to connect each TOF to a longer QWIIC cable so they could be placed as needed on the car, while the IMU would be connected with a shorter one. The below shows the schematic I decided on for connecting these sensors:

<img src='/images/mae4190/lab3/lab3_fig.png' width='700'>


## Battery
The first step was to solder cables to the 650 mAh battery, I used heat shrink to insulate each of these connections, shown below:

<img src='/images/mae4190/lab3/battery.jpg' width='600'>

The tricky thing about this is that, battery's + - is not same to the + - order on the Artimis board, I first solder cables wrong and the board started smoking, then I quick removed the battery, thank god it's not broke.


## First Time of Flight Sensor Setup

Firstly, I installed the SparkFun VL53L1X 4m laser distance library to interface with the TOF sensors. I then cut one of the longer QWIIC cables to allow for the sensor to be more flexible in placement, and referred to the data sheet to solder it. Specifically, blue ended up being the SDA line and yellow was the SCL line. The soldered sensor is shown below:

<img src='/images/mae4190/lab3/tof1.JPG' width='600'>

To verify that I connected this sensor correctly and that all of the solder joints were sound. I connected the QWIIC port on the Artemis to the QWIIC multiport breakout, and then the TOF sensor to this breakout:

<img src='/images/mae4190/lab3/onlyone.JPG' width='600'>

At first glance this address of 0x29 was surprising, as the datasheet for the TOF sensor said it had a default address of 0x52. However, looking at these in binary, 0x29 is 00101001 and 0x52 is 01010010, showing that the address was shifted right by one bit. This is because the least significant bit of the 0x52 address is actually used to set read or write, so the actual bts used for the i2c address are 0x29.

<img src='/images/mae4190/lab3/passone.png' width='700'>

## Second Time of Flight Sensor
I soldered the second time of flight sensor similarly to the first, but I additionally soldered a male connection wire to the XSHUT pin and a female wire to the Artemis Nano port A0. This allows for the Artemis to control the XSHUT pin via GPIO.

<img src='/images/mae4190/lab3/onlytwo.JPG' width='600'>

In order to run both sensors, on startup XSHUT needs to be held low to disable the second TOF sensor. Then, the address of the first TOF sensor is changed to 0x30 (one more than 0x29), and XSHUT is pulled high to enable the second TOF sensor. This is done in the following code. After this, each sensor can be used simulateously without problems.

<img src='/images/mae4190/lab3/passtwo.png' width='700'>

One subtlety with this initialization: after a **cold boot** (battery disconnected and reconnected), sensor 1 starts at its default address 0x29 and initialization proceeds as expected. However, after a **hot restart** (pressing reset or re-uploading firmware without cutting power), sensor 1 retains the remapped address 0x30 from the previous run. In that case `begin()`, which always scans for a device at 0x29, will fail to find it. The fix is simple: always perform a full power cycle before startup. Alternatively, a SOFT_RESET sequence can be written to sensor 1's address register at 0x30 to force it back to 0x29 before calling `begin()`.

## Time of Flight Sensors Testing

The next step was to ensure that the sensor was reading accurately. I made a setup with a measuring tape, using the provided white materials box as an object to detect and my computer to hold the sensor in one position, imaged below:

<img src='/images/mae4190/lab3/measure1.JPG' width='600'>
<img src='/images/mae4190/lab3/measure2.JPG' width='600'>

The TOF sensor could be set into several different distance modes, being short and long with tof1.setDistanceModeShort() and tof1.setDistanceModeLong(). Additionally, there was a medium mode, but this required an additional library to use. This short sensor mode is described in the datasheet as being less sensitive to ambient light but only effective to 1.3 meters, where the long would be effective to 4 meters. I decided to test both the short and long modes of the sensors between 100 millimeters and 1500 millimeters, testing for range, accuracy, and reliability. These sensor measurements were taken as averages of 256 TOF sensor readings to prove that the readings were repeatable. Notably, in the datasheet benchmarking this is done with 32 readings, but I used more to match the number of IMU readings I was taking. The below graph shows the long distance mode performance and short distance mode performance against a baseline tape measured distance.




---

[Back to MAE 4190](/mae4190/)
