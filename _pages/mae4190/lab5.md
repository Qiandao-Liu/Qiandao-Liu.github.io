---
layout: archive
title: "Lab 5: Linear PID and Linear Interpolation"
permalink: /mae4190/lab5/
author_profile: true
---

{% include base_path %}

Goal: drive the robot as fast as possible toward a wall and stop exactly 1 ft (304 mm) away using TOF sensor feedback. I tested P, PD, and PID controllers at three speed levels each and added linear extrapolation to decouple the PID loop rate from the sensor rate.

Setup: the robot starts 75 inches (1905 mm) from the wall every run. I placed a yoga mat against the wall as a crash buffer during high-speed tuning.

## Prelab

The BLE debugging flow has three stages. First, a Python `PID_START` command resets all state and starts the 10-second run on the Artemis. Second, the Artemis runs the controller, logs every TOF sample and PID sample into fixed arrays, and hard-stops on timeout even if BLE drops. Third, after Python sends `GET_PID_DATA`, the Artemis streams back all stored samples in `TOF|dist|extrap|time` and `PID|error|pwm|time` format, which Python parses into DataFrames.

Gains are tunable over BLE with `SET_PID_GAINS kp|ki|kd|setpoint` so I never had to reflash between tuning runs.

<details>
<summary>Arduino: PID_START handler</summary>
<div markdown="1">

```cpp
case PID_START:
{
    pid_e_pos = 0; pid_tof_pos = 0;
    pid_I = 0.0f; pid_dF = 0.0f; pid_last_e = 0.0f;
    tof_extrap_valid = false; tof_current = -1.0f;
    pid_start_ms = millis();
    pid_last_t   = millis();
    pid_running  = true;
    break;
}
```

</div>
</details>

<details>
<summary>Python: run and retrieve experiment</summary>
<div markdown="1">

```python
def run_pid_experiment(run_duration_s=10.0, label=""):
    ble.start_notify(ble.uuid['RX_STRING'], _pid_notify_handler)
    ble.send_command(CMD.PID_START, "")
    time.sleep(run_duration_s + 1.0)
    ble.send_command(CMD.PID_STOP, "")
    ble.send_command(CMD.GET_PID_DATA, "")
    # wait for PID_END marker, then parse TOF| and PID| lines
```

</div>
</details>

## TOF Sensor Configuration

I used the default TOF integration time, which gave real readings at roughly 10 Hz (100 ms per sample). The PID loop itself runs at ~112 Hz (8.9 ms per iteration) because it never blocks on sensor readiness. The extrapolation described below fills the gap between sensor readings. Lowering the integration time with `setProxIntegrationTime` would push the sensor rate higher at the cost of ranging accuracy, but 10 Hz was sufficient for 1905 mm approach distances.

## P Control

Starting point for KP: the firmware maps PID output to motor PWM as `PWM = 40 + (|output| / 200) * 160`. To cap the robot at 60 PWM during early tuning I needed a max PID output of 25, so `KP = 25 / 1700 ≈ 0.015` where 1700 mm is the typical starting error at 1905 mm start distance.

At 40 PWM the robot moved slowly and coasted to the wall rather than using reverse to brake. It often stopped 50-100 mm past the setpoint. At 80 PWM it needed to reverse and could stop cleanly in most runs. At 120 PWM the proportional term alone couldn't brake fast enough and the robot hit the wall every time. P-only final error in the logged run was -104 mm.

<img src='/images/mae4190/lab5/lab5_p_control.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab5/p_control.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">P control run at the main tuned setting.</div>
</div>

<div style="display:flex; gap:8px; flex-wrap:wrap; align-items:flex-start;">
  <div style="width:32%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab5/p_control_40pwm.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">P control at 40 PWM.</div>
  </div>
  <div style="width:32%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab5/p_control_80pwm.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">P control at 80 PWM.</div>
  </div>
  <div style="width:32%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab5/p_control_120pwm.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">P control at 120 PWM.</div>
  </div>
</div>

<img src='/images/mae4190/lab5/lab5_p_pwm_comparison.png' width='700'>

## PD Control

Adding derivative with `KD = 0.004` (ratio KD/KP ≈ 0.27) fixes the high-speed crash problem. The derivative term sees the large negative rate of change as the robot rushes toward the wall and applies braking force proportional to approach speed. A low-pass filter with α = 0.9 suppresses noise on the derivative: `pid_dF = 0.9 * pid_dF + 0.1 * d_raw`.

At 120 PWM the robot now brakes smoothly and stops within 21 mm of the setpoint. The motor output plot shows active reverse braking at high speed, which P-only couldn't do. PD performance at all three speed levels was already very close to PID, so the derivative term is doing most of the heavy lifting.

<img src='/images/mae4190/lab5/lab5_pd_control.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab5/pd_control.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">PD control run at the main tuned setting.</div>
</div>

<div style="display:flex; gap:8px; flex-wrap:wrap; align-items:flex-start;">
  <div style="width:32%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab5/pd_control_40pwm.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">PD control at 40 PWM.</div>
  </div>
  <div style="width:32%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab5/pd_control_80pwm.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">PD control at 80 PWM.</div>
  </div>
  <div style="width:32%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab5/pd_control_120pwm.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">PD control at 120 PWM.</div>
  </div>
</div>

<img src='/images/mae4190/lab5/lab5_pd_pwm_comparison.png' width='700'>

## PID Control

Adding `KI = 0.001` provides a gentle integrator that corrects the small residual steady-state error P and D can't fix on their own. The integral is clamped to ±1000 mm·s to prevent wind-up. PID final error in the logged run was -18 mm, compared to -104 mm for P and +21 mm for PD.

<details>
<summary>Arduino: full PID computation in handle_pid()</summary>
<div markdown="1">

```cpp
float e = tof_current - (float)pid_setpoint;

// Integral with wind-up clamp
pid_I += e * (float)dt / 1000.0f;
if (pid_I >  1000.0f) pid_I =  1000.0f;
if (pid_I < -1000.0f) pid_I = -1000.0f;

// Derivative with LPF (alpha = 0.9)
float d_raw = (e - pid_last_e) / ((float)dt / 1000.0f);
pid_dF = 0.9f * pid_dF + 0.1f * d_raw;
pid_last_e = e;

float output = pid_kp * e + pid_ki * pid_I + pid_kd * pid_dF;
output = constrain(output, -200.0f, 200.0f);

// Deadband mapping: skip the dead zone [0, 40 PWM]
if (fabsf(output) > 2.0f) {
    float mapped = 40.0f + (fabsf(output) / 200.0f) * 160.0f;
    pwm = (int)constrain(mapped, 40.0f, 200.0f);
    if (output > 0) motorsForward(pwm);
    else { motorsBackward(pwm); pwm = -pwm; }
} else {
    motorsStop();
}
```

</div>
</details>

For the robustness test I pushed the robot away from the wall mid-run. The PID controller corrected and returned to 304 mm.

<img src='/images/mae4190/lab5/lab5_pid_control.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab5/pid_contorl.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">PID control at the main tuned setting.</div>
</div>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab5/pid_robust_control.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">PID robustness test after pushing the robot away from the wall.</div>
</div>

<div style="display:flex; gap:8px; flex-wrap:wrap; align-items:flex-start;">
  <div style="width:32%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab5/pid_contorl_40pwm.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">PID control at 40 PWM.</div>
  </div>
  <div style="width:32%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab5/pid_contorl_80pwm.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">PID control at 80 PWM.</div>
  </div>
  <div style="width:32%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab5/pid_contorl_120pwm.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">PID control at 120 PWM.</div>
  </div>
</div>

<img src='/images/mae4190/lab5/lab5_pid_pwm_comparison.png' width='700'>

| Controller | KP | KI | KD | Final error |
|---|---|---|---|---|
| P | 0.015 | 0 | 0 | -104 mm |
| PD | 0.015 | 0 | 0.004 | +21 mm |
| PID | 0.015 | 0.001 | 0.004 | -18 mm |

## Linear Extrapolation

The TOF sensor delivers real data at 10 Hz, but the PID loop runs at 112 Hz. Without extrapolation, the derivative term sees a zero rate of change for roughly 90% of iterations because the error isn't updating. The fix is to estimate the current distance linearly from the last two real TOF readings.

<img src='/images/mae4190/lab5/extrapolation.png' width='700'>

Every time a new TOF value arrives, the Artemis computes the slope in mm/ms and stores it. Between readings it projects forward using `tof_current = tof_last_val + tof_slope * dt_since`. The `extrap` flag in each logged sample marks whether the value is real or estimated so the plots can distinguish them. The PID loop speed-up is 112 / 10 = 11.2x, meaning the derivative term gets a meaningful signal on every iteration instead of being stale 90% of the time.

<details>
<summary>Arduino: TOF extrapolation in handle_pid()</summary>
<div markdown="1">

```cpp
if (tofSensor1.checkForDataReady()) {
    float new_dist = (float)tofSensor1.getDistance();
    tofSensor1.clearInterrupt();
    unsigned long t_new = millis();
    if (tof_extrap_valid) {
        float dt_tof = (float)(t_new - tof_last_t_ms);
        if (dt_tof > 0.5f)
            tof_slope = (new_dist - tof_last_val) / dt_tof;
    }
    tof_last_val = new_dist;
    tof_last_t_ms = t_new;
    tof_extrap_valid = true;
    tof_current = new_dist;
} else if (tof_extrap_valid) {
    float dt_since = (float)(millis() - tof_last_t_ms);
    tof_current = tof_last_val + tof_slope * dt_since;
}
```

</div>
</details>

Meet with my cat Mulberry! 🐱

<img src='/images/mae4190/cats/cat10.png' width='400'>
<img src='/images/mae4190/cats/cat15.png' width='400'>

[Back to MAE 4190](/mae4190/)
