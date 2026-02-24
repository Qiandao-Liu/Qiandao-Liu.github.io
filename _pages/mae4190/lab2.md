---
layout: archive
title: "Lab 2: IMU"
permalink: /mae4190/lab2/
author_profile: true
---

{% include base_path %}

## Set up the IMU

### Hardware Connection

The ICM-20948 9-DOF IMU is connected to the Artemis Nano via the QWIIC connector (I2C). QWIIC interface supplies 3.3V power, GND, SDA and SCL.

<img src='/images/mae4190/lab2/imu_connection.jpg' width='600'>

After running the `Example1_Basics` sketch, the Serial Monitor confirms successful initialization with accelerometer and gyroscope readings updating in real-time:

<img src='/images/mae4190/lab2/pass_test_code.png' width='700'>

On startup LED blinks three times indicate the board is running.

<details>
<summary><strong>Arduino: LED startup blink</strong></summary>
<div markdown="1">

```cpp
// In setup()
for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
}
```

</div>
</details>

### AD0_VAL Discussion

`AD0_VAL` defines the last bit of the ICM-20948's I2C address:
- **AD0_VAL = 1** → I2C address `0x69` (ADR jumper **open**, default)
- **AD0_VAL = 0** → I2C address `0x68` (ADR jumper **closed**)

This allows two ICM-20948 sensors to coexist on the same I2C bus with different addresses. For single-IMU setup the ADR jumper is left open, so `AD0_VAL = 1` is used.

### Accelerometer and Gyroscope Data

When rotating or flipping the board, the accelerometer X/Y/Z readings change to reflect the component of gravity projected onto each axis. At rest flat on a table, `az ≈ 1000 mg` while `ax` and `ay` approach 0. Rotating 90° about the X-axis causes `ay` to swing from 0 to ±1000 mg. The gyroscope outputs angular velocity (dps) on each axis. Rapid accelerations produce large transient spikes in the accelerometer, while slow steady tilts show up clearly in the gyroscope as sustained non-zero readings.

<img src='/images/mae4190/lab2/accel_raw.png' width='700'>

<img src='/images/mae4190/lab2/gyro_raw.png' width='700'>

---

## Accelerometer

### Pitch and Roll Calculation

Pitch and roll are derived from accelerometer gravity projection using:

$$\text{pitch} = \arctan\!\left(\frac{a_x}{\sqrt{a_y^2 + a_z^2}}\right) \times \frac{180}{\pi}$$

$$\text{roll} = \arctan\!\left(\frac{a_y}{\sqrt{a_x^2 + a_z^2}}\right) \times \frac{180}{\pi}$$

Using `atan2` (from `math.h`) avoids quadrant ambiguity and handles the case where the denominator approaches zero.

<details>
<summary><strong>Arduino: pitch and roll from accelerometer</strong></summary>
<div markdown="1">

```cpp
#include <math.h>

float ax = myICM.accX();  // mg
float ay = myICM.accY();
float az = myICM.accZ();

float pitch_a = atan2(ax, sqrt(ay*ay + az*az)) * 180.0 / M_PI;
float roll_a  = atan2(ay, sqrt(ax*ax + az*az)) * 180.0 / M_PI;
```

</div>
</details>

### Output at -90, 0, 90 Degrees

Measurements at the three reference orientations, taken as the mean of 10 consecutive readings:

| Orientation | Pitch (measured) | Roll (measured) |
|-------------|------------------|-----------------|
| 0° (flat)   | -2.68°           | -0.63°          |
| +90°        | 86.23°           | 86.09°          |
| -90°        | -89.05°          | -85.51°         |

The accelerometer reads close to ±90° but not exactly, mainly because achieving a perfect right-angle by hand is difficult. The flat (0°) reading shows a small offset bias of about -2.7° for pitch and -0.6° for roll.

### Accelerometer Accuracy and Two-Point Calibration

A two-point calibration was performed using the ±90° measurements as reference endpoints. The calibration computes a linear scale and offset such that the corrected output matches the expected output:

$$\text{corrected} = \text{scale} \times \text{measured} + \text{offset}$$

**Pitch Calibration:** scale = 1.027, offset = +1.45°
**Roll Calibration:** scale = 1.049, offset = −0.31°

After calibration, errors at the extreme angles are reduced to < 1°. The remaining error at 0° is within the expected bias of a consumer-grade MEMS sensor.

<details>
<summary><strong>Python: two-point calibration function</strong></summary>
<div markdown="1">

```python
def two_point_calibration(measured_low, measured_high,
                           expected_low=-90.0, expected_high=90.0):
    scale  = (expected_high - expected_low) / (measured_high - measured_low)
    offset = expected_high - scale * measured_high
    print(f"Scale: {scale:.6f}, Offset: {offset:.4f}")
    return scale, offset

# Pitch: measured -89.05 at -90, 86.23 at +90
pitch_scale, pitch_offset = two_point_calibration(-89.05, 86.23)
# Roll: measured -85.51 at -90, 86.09 at +90
roll_scale, roll_offset   = two_point_calibration(-85.51, 86.09)
```

</div>
</details>

### Noise and Frequency Spectrum Analysis

FFT analysis was performed on stationary accelerometer pitch data (sampling rate ≈ 342.7 Hz, Nyquist ≈ 171.4 Hz):

<img src='/images/mae4190/lab2/fft_stationary.png' width='700'>

In the stationary case, noise energy is concentrated below 10 Hz with no strong peaks, the sensor is well-behaved at rest. To induce vibration noise, the table was tapped gently during a second recording:

<img src='/images/mae4190/lab2/fft_vibration.png' width='700'>

Tapping the table introduces broadband noise spanning multiple frequency bands. The useful orientation signal (slow tilts) lives below ~5 Hz, while the vibration energy appears primarily above 10 Hz. This motivates a low-pass filter with a cutoff in the 5–10 Hz range.

<details>
<summary><strong>Python: FFT analysis</strong></summary>
<div markdown="1">

```python
import numpy as np
import matplotlib.pyplot as plt

def plot_fft(df, col, title=None):
    sig = df[col].values
    n   = len(sig)
    dt  = np.mean(np.diff(df['time_s'].values))
    fs  = 1.0 / dt

    fft_vals   = np.fft.rfft(sig - np.mean(sig))
    freqs      = np.fft.rfftfreq(n, d=dt)
    magnitudes = np.abs(fft_vals) * 2.0 / n

    plt.figure(figsize=(10, 4))
    plt.plot(freqs, magnitudes)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.title(title or f'FFT of {col}')
    plt.grid(True, alpha=0.3)
    plt.xlim(0, fs/2)
    plt.tight_layout()
    plt.show()
    print(f"Sample rate: {fs:.1f} Hz,  Nyquist: {fs/2:.1f} Hz")
```

</div>
</details>

### Low-Pass Filter Implementation

The discrete first-order IIR low-pass filter is:

$$y[n] = \alpha \cdot x[n] + (1 - \alpha) \cdot y[n-1], \quad \alpha = \frac{d_t}{d_t + \frac{1}{2\pi f_c}}$$

The alpha value for several candidate cutoff frequencies (at the measured 2.92 ms sample period):

| Cutoff Frequency | Alpha  |
|------------------|--------|
| 1 Hz             | 0.018  |
| 2 Hz             | 0.035  |
| 5 Hz             | 0.084  |
| 10 Hz            | 0.155  |
| 20 Hz            | 0.268  |

**Selected: α = 0.2 (≈ 10 Hz cutoff).** This removes the high-frequency vibration noise visible in the FFT while preserving the full dynamic range of typical orientation changes (< 5 Hz). A lower cutoff (e.g., 1 Hz) would add visible lag during fast tilts; a higher cutoff (e.g., 20 Hz) would leave more vibration noise.

<details>
<summary><strong>Arduino: low-pass filter on accelerometer pitch/roll</strong></summary>
<div markdown="1">

```cpp
float alpha_lpf = 0.2;  // ~10 Hz cutoff
// Persistent state
float pitch_a_lpf = 0.0, roll_a_lpf = 0.0;

pitch_a_lpf = alpha_lpf * pitch_a + (1.0 - alpha_lpf) * pitch_a_lpf;
roll_a_lpf  = alpha_lpf * roll_a  + (1.0 - alpha_lpf) * roll_a_lpf;
```

</div>
</details>

---

## Gyroscope

### Pitch, Roll, and Yaw from Gyroscope Integration

The gyroscope outputs angular velocity (dps). Integrating over time gives angle estimates:

$$\theta[n] = \theta[n-1] + \omega \cdot d_t$$

- **Pitch** (rotation about Y): integrated from `gyrX()`
- **Roll** (rotation about X): integrated from `gyrY()`
- **Yaw** (rotation about Z): integrated from `gyrZ()`

<details>
<summary><strong>Arduino: gyroscope angle integration</strong></summary>
<div markdown="1">

```cpp
float gx = myICM.gyrX();  // dps
float gy = myICM.gyrY();
float gz = myICM.gyrZ();

unsigned long now = millis();
float dt = (now - lastIMUTime) / 1000.0;
if (dt <= 0) dt = 0.001;  // guard against zero division
lastIMUTime = now;

pitch_g += gx * dt;
roll_g  += gy * dt;
yaw_g   += gz * dt;
```

</div>
</details>

### Comparison with Accelerometer

<img src='/images/mae4190/lab2/filter_comparison.png' width='700'>

Key differences between the methods:

- **Gyro integration** responds instantly to motion but accumulates bias drift — even when held perfectly still, the angle slowly walks away from zero over tens of seconds.
- **Raw accelerometer** has no drift but is noisy, especially during vibration or rapid motion. It also cannot measure yaw.
- **LPF accelerometer** smooths out vibration but adds lag proportional to the filter cutoff.

**Effect of sampling frequency on gyro accuracy:** Lowering the sampling rate increases `dt`, which amplifies the integration error for any given angular velocity. At the measured ~343 Hz, gyro integration is accurate for short durations (< 10 s). At 50 Hz, the integration error for a fast rotation (e.g., 180°/s) increases by roughly 7× compared to 343 Hz. For the complementary filter to be effective, sampling should be kept high.

### Complementary Filter

The complementary filter fuses the two sensors — using the gyroscope for short-term accuracy and the accelerometer for long-term correction:

$$\theta[n] = (1-\alpha)\bigl(\theta[n-1] + \omega \cdot d_t\bigr) + \alpha \cdot \theta_\text{accel}$$

<details>
<summary><strong>Arduino: complementary filter</strong></summary>
<div markdown="1">

```cpp
float alpha_comp = 0.05;  // 5% accel weight, 95% gyro weight
float pitch_comp = 0.0, roll_comp = 0.0;

pitch_comp = (1.0 - alpha_comp) * (pitch_comp + gx * dt) + alpha_comp * pitch_a;
roll_comp  = (1.0 - alpha_comp) * (roll_comp  + gy * dt) + alpha_comp * roll_a;
```

</div>
</details>

**Drift test** — IMU held stationary for ~10 seconds:

<img src='/images/mae4190/lab2/drift_test.png' width='700'>

The gyro integral drifts continuously; the complementary filter stays within ~1° of the true angle because the 5% accelerometer weighting slowly corrects any accumulated bias.

**Vibration rejection test** — table tapped while recording:

<img src='/images/mae4190/lab2/vibration_test.png' width='700'>

The raw accelerometer spikes by several degrees during each tap. The complementary filter's dominant gyroscope weight (95%) suppresses these transients, maintaining a smooth output.

**Design choices:**
- **α_comp = 0.05**: Balances drift correction (~20 s time constant) against vibration rejection. Higher α (e.g., 0.2) corrects drift faster but lets more vibration noise through.
- **α_lpf = 0.2**: Pre-filters the accelerometer input to the complementary filter, further reducing vibration sensitivity.
- **Sample rate ≈ 343 Hz**: Fast enough to accurately integrate sudden angular accelerations; loop runs faster than the IMU produces new data (see below).

---

## Sample Data

### Speed of Sampling

With all `Serial.print` statements and `delay()` calls removed from the main loop, the Artemis checks `myICM.dataReady()` on every iteration and stores data only when a new sample is available. The measured throughput is **~343 samples/second (~2.9 ms/sample)**.

The Artemis main loop runs faster than the IMU's internal ODR (Output Data Rate). `dataReady()` returns false on most loop iterations, so the loop does not block — it just polls and moves on. This non-blocking design is critical: the same loop simultaneously handles BLE `read_data()` without introducing timing jitter.

<details>
<summary><strong>Arduino: non-blocking main loop</strong></summary>
<div markdown="1">

```cpp
void loop() {
    BLEDevice central = BLE.central();
    if (central) {
        while (central.connected()) {
            write_data();   // period heartbeat
            read_data();    // handle any incoming BLE commands

            // Non-blocking IMU collection only store when data is ready
            if (collectingIMU) {
                record_imu_data();  // returns immediately if !dataReady()
            }
        }
    }
}

void record_imu_data() {
    if (!imuInitialized || imuArrayFull) return;
    if (!myICM.dataReady()) return;  // non-blocking check

    myICM.getAGMT();
    // ... compute angles, store in arrays ...
}
```

</div>
</details>

### Data Storage Design

Separate arrays are used for each quantity rather than a single large interleaved array. This makes indexing clear and avoids struct padding/alignment overhead:

```cpp
#define MAX_IMU_SIZE 2000

unsigned long imuTimeStamps[MAX_IMU_SIZE];   // 8 kB
float imuPitchA[MAX_IMU_SIZE];               // 8 kB
float imuRollA[MAX_IMU_SIZE];                // 8 kB
float imuPitchALpf[MAX_IMU_SIZE];            // 8 kB
float imuRollALpf[MAX_IMU_SIZE];             // 8 kB
float imuPitchG[MAX_IMU_SIZE];               // 8 kB
float imuRollG[MAX_IMU_SIZE];                // 8 kB
float imuYawG[MAX_IMU_SIZE];                 // 8 kB
float imuPitchComp[MAX_IMU_SIZE];            // 8 kB
float imuRollComp[MAX_IMU_SIZE];             // 8 kB
// Total 2000 samples: ~80 kB
```

**Data type choice: `float` (4 bytes):** Angle values range from −180° to +180° with sub-degree precision requirements. `int16_t` would save memory but loses the fractional degrees needed for filter accuracy. `double` (8 bytes) provides no practical benefit over `float` for orientation at these noise levels.

**Memory budget:** The Artemis has 384 kB RAM. With ~80 kB for IMU arrays, ~50 kB for code/stack/BLE buffers, there is roughly 250 kB available for data. Storing all 16 fields as floats cost 64 bytes/sample. That allows ~3900 samples about 11.4 seconds at 343 Hz. For the compact 5-field transmit format (timestamp + pitch_a + roll_a + pitch_comp + roll_comp = 20 bytes/sample) the limit is ~12,500 samples (~36 seconds at 343 Hz).

### 5+ Seconds of IMU Data via Bluetooth

After recording, the `SEND_IMU_DATA` command transmits the stored data. To stay within BLE bandwidth limits, every 3rd sample is sent (downsampling to ~115 Hz effective), with a 30 ms delay between packets to prevent buffer overflow:

<details>
<summary><strong>Arduino: SEND_IMU_DATA command</strong></summary>
<div markdown="1">

```cpp
case SEND_IMU_DATA:
{
    int limit = imuArrayFull ? MAX_IMU_SIZE : imuIndex;
    int step  = 3;  // downsample ~115 Hz effective

    for (int i = 0; i < limit; i += step) {
        tx_estring_value.clear();
        tx_estring_value.append((int)imuTimeStamps[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(imuPitchA[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(imuRollA[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(imuPitchComp[i]);
        tx_estring_value.append("|");
        tx_estring_value.append(imuRollComp[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        delay(30);  // throttle to prevent BLE buffer overflow
    }
    break;
}
```

</div>
</details>

<details>
<summary><strong>Python: notification handler and data collection</strong></summary>
<div markdown="1">

```python
imu_data_buffer = []

def imu_notification_handler(sender, data):
    msg = data.decode('utf-8')
    imu_data_buffer.append(msg)

def collect_imu_data(duration_s=5):
    global imu_data_buffer
    imu_data_buffer = []

    ble.start_notify(ble.uuid['RX_STRING'], imu_notification_handler)
    ble.send_command(CMD.START_IMU_RECORDING, "")
    time.sleep(duration_s)
    ble.send_command(CMD.STOP_IMU_RECORDING, "")
    time.sleep(0.5)
    ble.send_command(CMD.SEND_IMU_DATA, "")

    # wait for transfer to complete
    time.sleep(duration_s * 0.035 * 100 + 5)
    ble.stop_notify(ble.uuid['RX_STRING'])

    # parse pipe-delimited format: time_ms|pitch_a|roll_a|pitch_comp|roll_comp
    rows = []
    for line in imu_data_buffer:
        parts = line.split('|')
        if len(parts) == 5:
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                pass
    return pd.DataFrame(rows,
        columns=['time_ms', 'pitch_a', 'roll_a', 'pitch_comp', 'roll_comp'])
```

</div>
</details>

Successfully transmitted **5.34 seconds** of IMU data over BLE (667 samples at an effective 124.8 Hz):

<img src='/images/mae4190/lab2/5sec_data.png' width='700'>

---

## Record a Stunt

### Stunt 1: Drift and backhit

<img src='/images/mae4190/lab2/vid_1.gif' width='600'>

### Stunt 2: Turning

<img src='/images/mae4190/lab2/vid_2.gif' width='600'>

### Stunt 3: Flips

<img src='/images/mae4190/lab2/vid_3.gif' width='600'>

The car accelerates very quickly, full throttle from rest produces a noticeable forward lurch. Turning at high speed induces sideways drift, especially on smooth floors. The car can flip end-over-end with a sudden reverse input at speed. These dynamics suggest the IMU will need to capture super sharp transients during autonomous operation. The complementary filter's high gyroscope weighting will be important, for maintaining a stable angle estimate through these vibration-heavy maneuvers.

---

## Discussion

### Method Comparison

| Method | Strengths | Weaknesses | Best use |
|---|---|---|---|
| Raw accelerometer | No drift, absolute reference | Noisy, no yaw | Slow static orientation |
| LPF accelerometer | Reduced noise | Lag at high α, no yaw | Background correction |
| Gyroscope integration | Fast, captures yaw | Drift over time | Short-duration dynamics |
| Complementary filter | Stable + responsive + no drift | Cannot measure yaw | Real-time robot state |

### Key Learnings

1. **Accelerometer alone is insufficient:** vibration during RC car operation introduces errors of several degrees; the LPF helps but cannot fully eliminate this.
2. **Gyroscope alone drifts:** bias integration makes it unreliable beyond ~30 s without correction.
3. **Complementary filter is the right tool:** the 95%/5% gyro/accel split gives responsive, drift-free angle estimates robust to the car's vibrations.
4. **BLE is the bottleneck, not sampling:** the IMU can sample at 343 Hz but BLE reliable throughput is ~30 packets/s; local buffering and batch transfer are essential.
5. **Non-blocking loop design matters:** checking `dataReady()` without waiting ensures the main loop stays responsive to BLE commands during recording.

---

[Back to MAE 4190](/mae4190/)
