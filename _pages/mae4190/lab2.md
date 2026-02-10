---
layout: archive
title: "Lab 2: IMU"
permalink: /mae4190/lab2/
author_profile: true
---

{% include base_path %}

## Set up the IMU

### Hardware Connection

ICM-20948 IMU is connected to Artemis Nano. Provides 9-DOF sensing.

<img src='/images/mae4190/lab2/imu_connection.jpg' width='600'>

The Serial Monitor confirms successful initialization with accelerometer and gyroscope readings updating in real-time!

<img src='/images/mae4190/lab2/pass_test_code.png' width='700'>

### AD0_VAL Discussion

The `AD0_VAL` defines the I2C address of the IMU:
- **AD0_VAL = 1**: I2C address is 0x69 (ADR jumper open - default)
- **AD0_VAL = 0**: I2C address is 0x68 (ADR jumper closed)

This allows two ICM-20948 sensors on the same I2C bus with different addresses. For our single-IMU setup, `AD0_VAL = 1` is used.

### Accelerometer and Gyroscope Data

The IMU outputs:
- **Accelerometer**: Measures linear acceleration in mg (milli-g) on X, Y, Z axes
- **Gyroscope**: Measures angular velocity in dps (degrees per second) on X, Y, Z axes

<img src='/images/mae4190/lab2/accel_raw.png' width='700'>

<img src='/images/mae4190/lab2/gyro_raw.png' width='700'>

---

## Accelerometer

### Pitch and Roll Calculation

Pitch and roll angles are calculated from accelerometer data using:

$$\text{pitch} = \arctan\left(\frac{a_x}{\sqrt{a_y^2 + a_z^2}}\right)$$

$$\text{roll} = \arctan\left(\frac{a_y}{\sqrt{a_x^2 + a_z^2}}\right)$$

### Output at -90, 0, 90 Degrees

Measurements at three orientations for pitch and roll:

| Orientation | Pitch (measured) | Roll (measured) |
|-------------|------------------|-----------------|
| 0° (flat)   | -2.68°           | -0.63°          |
| +90°        | 86.23°           | 86.09°          |
| -90°        | -89.05°          | -85.51°         |

### Accelerometer Accuracy

Using two-point calibration to compute correction parameters:

**Pitch Calibration:**
- Scale: 1.027
- Offset: 1.45°
- Corrected formula: `pitch_corrected = 1.027 * pitch_measured + 1.45`

**Roll Calibration:**
- Scale: 1.049
- Offset: -0.31°
- Corrected formula: `roll_corrected = 1.049 * roll_measured - 0.31`

The accelerometer shows good accuracy with errors less than 4° at extreme angles. The primary source of error is the difficulty in achieving exact 90° orientations during calibration.

### Noise and Frequency Spectrum Analysis

FFT analysis was performed on stationary accelerometer data to characterize noise:

<img src='/images/mae4190/lab2/fft_stationary.png' width='700'>

**Stationary Noise Analysis:**
- Sampling rate: 342.7 Hz
- Nyquist frequency: 171.4 Hz
- Most noise energy concentrated below 10 Hz
- No significant high-frequency peaks in stationary condition

<img src='/images/mae4190/lab2/fft_vibration.png' width='700'>

**Vibration Noise Analysis:**
- Tapping table introduce broadband noise
- Vibration energy visible across multiple frequencies

### Low-Pass Filter Design

Based on FFT analysis a low-pass filter with cutoff frequency of 5-10 Hz effectively removes high-frequency noise while preserving orientation signals.

The filter alpha is computed as:
```
alpha = dt / (dt + 1/(2*pi*fc))
```

| Cutoff Frequency | Alpha |
|------------------|-------|
| 1 Hz             | 0.018 |
| 2 Hz             | 0.035 |
| 5 Hz             | 0.084 |
| 10 Hz            | 0.155 |
| 20 Hz            | 0.268 |

We selected **alpha = 0.2** (approximately 10 Hz cutoff) for the low-pass filter.

---

## Gyroscope

### Pitch, Roll, and Yaw from Gyroscope

Gyroscope provides angular velocity which is integrated to obtain angles:

$$\theta(t) = \theta(t-1) + \omega \cdot dt$$

Where:
- Pitch (rotation about Y-axis): Integrated from gx
- Roll (rotation about X-axis): Integrated from gy
- Yaw (rotation about Z-axis): Integrated from gz

**Coordinate System:**
- X-axis: Points forward
- Y-axis: Points left
- Z-axis: Points up

### Complementary Filter

The complementary filter combines accelerometer and gyroscope data:

$$\theta = (1-\alpha) \cdot (\theta_{prev} + \omega \cdot dt) + \alpha \cdot \theta_{accel}$$

With α = 0.05:
- **95%** weight on gyroscope (fast response, no vibration sensitivity)
- **5%** weight on accelerometer (absolute reference, no drift)

<img src='/images/mae4190/lab2/filter_comparison.png' width='700'>

### Complementary Filter Accuracy and Stability

**Drift Test (IMU held stationary):**

<img src='/images/mae4190/lab2/drift_test.png' width='700'>

The gyroscope integration drifts over time due to bias, while the complementary filter remains stable by using the accelerometer as an absolute reference.

**Vibration Rejection Test:**

<img src='/images/mae4190/lab2/vibration_test.png' width='700'>

The raw accelerometer shows significant noise during vibration, while the complementary filter maintains a smooth output by primarily trusting the gyroscope for short-term changes.

### Design Choices

1. Alpha = 0.05: Chosen to provide good drift correction while maintaining vibration rejection
2. Sample rate ~350 Hz: Fast enough to capture rapid motions
3. Low-pass filter alpha = 0.2: Removes high-frequency noise from accelerometer

---

## Sample Data

### Speed of Sampling

The IMU data collection achieves:
- Sampling rate: 342.7 Hz (~2.9 ms per sample)
- This is sufficient for capturing fast robot motions

The sampling rate is limited by:
1. I2C communication speed (400 kHz)
2. IMU internal data rate
3. Arduino loop execution time

### Time-Stamped IMU Data Storage

Data is stored in arrays on the Artemis board:

```cpp
#define MAX_IMU_SIZE 2000
unsigned long imuTimeStamps[MAX_IMU_SIZE];
float imuPitchA[MAX_IMU_SIZE], imuRollA[MAX_IMU_SIZE];
float imuPitchComp[MAX_IMU_SIZE], imuRollComp[MAX_IMU_SIZE];
```

Each sample includes:
- Timestamp (ms)
- Raw accelerometer pitch/roll
- Complementary filter pitch/roll

### 5+ Seconds of IMU Data via Bluetooth

Successfully transmitted **5.34 seconds** of IMU data over BLE:

<img src='/images/mae4190/lab2/5sec_data.png' width='700'>

**Results:**
- Total samples: 667
- Duration: 5.34 seconds
- Effective sample rate: 124.8 Hz (after downsampling for reliable BLE transfer)

**BLE Optimization:**
- Original data rate (~350 Hz) exceeded BLE bandwidth
- Implemented 3x downsampling for reliable transmission
- Added 30ms delay between packets to prevent buffer overflow

---

## Record a Stunt

Testing the RC car to observe its dynamics before integrating IMU data:

### Stunt 1: Drift and backhit

<img src='/images/mae4190/lab2/vid_1.gif' width='600'>

### Stunt 2: Turning

<img src='/images/mae4190/lab2/vid_2.gif' width='600'>

### Stunt 3: Stunts Flips

<img src='/images/mae4190/lab2/vid_3.gif' width='600'>

---

## Discussion

### Key Learnings

1. Accelerometer alone is noisy - Vibrations cause significant measurement errors
2. Gyroscope alone drifts - Integration accumulates bias over time
3. Complementary filter combines benefits - Stable responsive angle estimation
4. BLE bandwidth limited - Must optimize data transfer for high-rate sensors
5. Downsampling enables reliable transfer - Trade-off between resolution and reliability

---

[Back to MAE 4190](/mae4190/)
