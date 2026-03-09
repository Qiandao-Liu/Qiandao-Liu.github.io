---
layout: archive
title: "Lab 3: Time of Flight Sensors"
permalink: /mae4190/lab3/
author_profile: true
---

{% include base_path %}

## Prelab

The goal of this lab is to set up two VL53L1X time-of-flight sensors and get them ready to be mounted on the RC car for future labs. The sensors work by emitting infrared light and measuring how long it takes to bounce back, which gives a distance estimate.

### I2C Address

The VL53L1X has a default I2C address of **0x52** (per the datasheet), but the Arduino Wire library uses 7-bit addressing, so it shows up as **0x29** (0x52 >> 1). Both sensors share this same hardwired address, so we can't use them simultaneously without doing something about it.

### Handling Two Sensors on the Same Bus

There are two common approaches:

1. **Toggle XSHUT at runtime** — keep one sensor off while reading the other, then swap. Simple but you can only ever read one sensor at a time, which cuts your effective sample rate in half.
2. **Remap one address at startup** — bring up sensor 1 first, change its I2C address to 0x30, then bring up sensor 2 at 0x29. Both sensors live on the bus simultaneously with no conflicts.

I went with option 2. But there's a catch: you need **XSHUT control on both sensors** so you can hold them both in hardware reset at power-on, then release them one at a time. Without this, a hot-restart (re-upload without power cycling) leaves sensor 1 sitting at 0x30 already, so `begin()` (which scans at 0x29) fails.

So I soldered XSHUT wires for both sensors: **sensor 1 XSHUT → A1 (GPIO 1)** and **sensor 2 XSHUT → A0 (GPIO 0)**. Both pins go LOW in `setup()` to hardware-reset both sensors, then they're released sequentially in `init_tof_sensors()`.

### Sensor Placement

One sensor faces **forward** on the front of the car — that's the main one since the robot mostly drives forward. The second sensor is mounted on the **right side** to detect walls during mapping tasks in later labs.

Scenarios where the robot will **miss obstacles**:
- Objects behind the robot (no rear sensor)
- Objects on the left side
- Very thin objects (like wire legs of a chair) that the IR beam passes between
- Very dark/matte black surfaces that absorb IR and reflect little back
- Transparent obstacles like glass

### Wiring Plan

<img src='/images/mae4190/lab3/lab3_fig.png' width='700'>

<img src='/images/mae4190/lab3/lab3_fig2.png' width='700'>

Long QWIIC cables go to the ToF sensors so they can reach their mounting positions on the car. The IMU uses a shorter cable since it lives close to the Artemis. XSHUT wires are soldered permanently to each sensor and connect via female-to-female jumper wires to A0/A1 on the Artemis (detachable for debugging).

---

## Battery

First step was to solder the JST cable to the 650 mAh battery one wire at a time (cutting both at once = short circuit = bad day). Used heat shrink to insulate each joint.

<img src='/images/mae4190/lab3/battery.jpg' width='600'>

One thing to watch out for: the battery wire polarity doesn't necessarily match the color convention on the Artemis side. I got it wrong on the first try and the board started smoking. Pulled the battery fast, checked polarity with a multimeter, re-soldered correctly, and everything was fine.

---

## Task 4: First ToF Sensor Setup

Installed the SparkFun VL53L1X 4m library, cut a long QWIIC cable, and soldered it to sensor 1. Blue = SDA, yellow = SCL per the datasheet.

<img src='/images/mae4190/lab3/tof1.JPG' width='600'>

Connected it to the QWIIC breakout board on the Artemis to verify the solder joints:

<img src='/images/mae4190/lab3/onlyone.JPG' width='600'>

---

## Task 5: I2C Address Scan

Ran the I2C scanner example. The sensor shows up at **0x29**, not 0x52 as the datasheet says. This makes sense because the Arduino Wire library uses 7-bit addresses, and 0x52 in 8-bit (where the LSB is the R/W bit) becomes 0x29 in 7-bit. So it's correct, just a different convention.

<img src='/images/mae4190/lab3/passone.png' width='700'>

---

## Task 6: Distance Mode Comparison

The VL53L1X has two main distance modes:

| Mode | Max Range | Timing Budget | Ambient Light Sensitivity |
|------|-----------|---------------|--------------------------|
| **Short** | ~1.3 m | ≤ 20 ms | Low (more robust) |
| **Long** | ~4 m | ≥ 33 ms | Higher |

Short mode is more reliable in bright environments but tops out at 1.3 m. Long mode can reach 4 m but starts getting noisy in direct sunlight or with reflective surfaces past ~2 m.

For this robot I'm using **Long mode** as the default. The indoor arenas we operate in don't have intense ambient IR, and having 4 m range is more useful than the extra noise immunity. Short mode is still available via BLE command (`SET_TOF_MODE 0`) for comparison.

One important thing I learned: you **must** call `stopRanging()` before switching modes. If you call `setDistanceMode()` while the sensor is actively ranging, it corrupts internal timing registers and causes a ~130 mm systematic offset on every subsequent reading. That was a fun bug to track down.

---

## Task 7: Characterization — Range, Accuracy, Repeatability

I collected 50 single-shot readings at each of 5 distances (100, 500, 900, 1300, 1700 mm) for both sensors in both Short and Long mode.

### Sensor 1 Accuracy

<img src='/images/mae4190/lab3/tof_accuracy_sensor1.png' width='800'>

Sensor 1 is consistently around 15–30 mm under the true value in Long mode, which is a known mounting offset from the protective lens cover. Short mode performs similarly within range but maxes out around 1300 mm (the 1700 mm readings jumped to ~2200 mm, way out of range).

### Sensor 2 Accuracy

<img src='/images/mae4190/lab3/tof_accuracy_sensor2.png' width='800'>

Sensor 2 is more accurate in absolute terms — error is within ±15 mm across 100–1300 mm. Same blowup at 1700 mm in both modes (exceeds reliable range in indoor ambient light).

### Repeatability

50 readings per distance gives a clear picture of spread. σ is under 2 mm for all distances up to 1300 mm, which is good.

<img src='/images/mae4190/lab3/tof_repeatability.png' width='900'>

The 1700 mm histograms show clearly bimodal or very spread distributions — the sensor is unreliable past its rated range. In practice I'll keep the robot within 1.5 m of obstacles.

---

## Task 8: Two ToF Sensors Simultaneously

Both XSHUT wires are wired up — sensor 1 to A1, sensor 2 to A0. The init sequence holds both LOW at startup (hardware reset), then releases them one at a time:

<details>
<summary><strong>Arduino: init_tof_sensors() — dual XSHUT hardware reset sequence</strong></summary>
<div markdown="1">

```cpp
#define SHUTDOWN_PIN_1  1   // A1 → sensor 1 XSHUT
#define SHUTDOWN_PIN    0   // A0 → sensor 2 XSHUT
#define TOF1_ADDR     0x30  // sensor 1 remapped here

void init_tof_sensors() {
    // Both XSHUT pins are already LOW from setup() — sensors in hardware reset.
    // Release sensor 1 first so it boots exclusively at 0x29.
    digitalWrite(SHUTDOWN_PIN_1, HIGH);
    delay(10);  // VL53L1X boot time

    if (tofSensor1.begin() == 0) {
        tofSensor1.setI2CAddress(TOF1_ADDR);  // remap → 0x30
        tofSensor1.setDistanceModeLong();
        tofSensor1.startRanging();
        tof1Ready = true;
        Serial.println("ToF sensor 1 OK");
    } else {
        Serial.println("ToF sensor 1 FAILED");
    }

    // Now release sensor 2; sensor 1 is already at 0x30, no collision.
    digitalWrite(SHUTDOWN_PIN, HIGH);
    delay(10);

    if (tofSensor2.begin() == 0) {
        tofSensor2.setDistanceModeLong();
        tofSensor2.startRanging();
        tof2Ready = true;
        Serial.println("ToF sensor 2 OK");
    } else {
        Serial.println("ToF sensor 2 FAILED");
    }
}
```

</div>
</details>

After this, both sensors are live on the I2C bus — sensor 1 at 0x30, sensor 2 at 0x29. The hot-restart problem is also fixed now: since both XSHUT pins are driven LOW in `setup()` every time the Artemis boots, the sensors are always in a clean reset state regardless of what address they had in the previous run.

<img src='/images/mae4190/lab3/passtwo.png' width='700'>

<img src='/images/mae4190/lab3/2tof并行工作截图.png' width='700'>

Both sensors returning valid independent readings simultaneously.

---

## Task 9: Non-Blocking Loop Speed

The key constraint for future labs is that the main loop can't stall waiting for sensor data. The `checkForDataReady()` approach lets the loop keep running at full speed and only processes new data when it's actually available.

<details>
<summary><strong>Arduino: non-blocking record_tof_data() called from loop()</strong></summary>
<div markdown="1">

```cpp
void record_tof_data() {
    if (!tof1Ready || tofArrayFull) return;
    if (!tofSensor1.checkForDataReady()) return;  // skip if not ready — no blocking

    tofTimeStamps[tofIndex] = millis();
    tofDist1[tofIndex] = (int16_t)tofSensor1.getDistance();
    tofSensor1.clearInterrupt();

    // Read sensor 2 opportunistically — it might not be ready every cycle
    if (tof2Ready && tofSensor2.checkForDataReady()) {
        tofDist2[tofIndex] = (int16_t)tofSensor2.getDistance();
        tofSensor2.clearInterrupt();
    } else {
        tofDist2[tofIndex] = -1;  // sensor 2 not ready this cycle
    }

    tofIndex++;
    if (tofIndex >= MAX_TOF_SIZE) {
        tofArrayFull = true;
        collectingTOF = false;
    }
}
```

</div>
</details>

The loop itself runs much faster than the sensor — each iteration is on the order of tens of microseconds. The **limiting factor is the ToF sensor ranging time**: Long mode uses a 33 ms timing budget, which caps the maximum data rate at ~30 Hz. In practice the diagnostic output shows around **10 Hz** per sensor, which is consistent with what I'm seeing (the sensor needs time to integrate enough photons for a reliable reading).

The IMU runs faster since it just reads an I2C register, but it's ultimately bounded by the sensor's ODR (~100 Hz).

---

## Task 10/11: Distance vs Time (BLE Transfer)

Sent `START_TOF_RECORDING`, waited ~6 seconds while moving the sensors, then retrieved data over BLE.

<details>
<summary><strong>Arduino: SEND_TOF_DATA — streaming stored samples over BLE</strong></summary>
<div markdown="1">

```cpp
case SEND_TOF_DATA: {
    int limit = tofArrayFull ? MAX_TOF_SIZE : tofIndex;
    for (int i = 0; i < limit; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("T|");
        tx_estring_value.append((int)tofTimeStamps[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)tofDist1[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)tofDist2[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        delay(10);
    }
    break;
}
```

</div>
</details>

<img src='/images/mae4190/lab3/tof_distance_vs_time.png' width='800'>

Both sensors tracking distance changes over time. The gaps in sensor 2 are where it wasn't ready at the exact moment sensor 1 fired — normal behavior since they run asynchronously.

---

## Task 12: IMU Angle vs Time

<img src='/images/mae4190/lab3/imu_angle_vs_time.png' width='800'>

Pitch and roll from the complementary filter (α = 0.05) vs raw accelerometer. The complementary filter smooths out the accelerometer noise while avoiding gyroscope drift. IMU samples at a much higher rate than the ToF (~100 Hz) since there's no long integration window.

---

## Combined: ToF + IMU on One Plot

Both datasets on one figure — left axis is distance (mm), right axis is angle (degrees). The IMU x-axis extends further because it samples faster.

<img src='/images/mae4190/lab3/tof_imu_combined.png' width='900'>

---
Meet my cat Mulberry! 🐱

<img src='/images/mae4190/cats/cat4.png' width='400'>
<img src='/images/mae4190/cats/cat11.png' width='400'>

---

[Back to MAE 4190](/mae4190/)
