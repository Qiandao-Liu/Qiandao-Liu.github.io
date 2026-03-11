---
layout: archive
title: "Lab 6: Orientation Control"
permalink: /mae4190/lab6/
author_profile: true
---

{% include base_path %}

Goal: control the yaw angle of the robot in place using gyroscope feedback and a PID controller. I tested P, PD, and PID configurations, addressed derivative kick with a low-pass filter, and demonstrated real-time setpoint changes over BLE.

## Prelab

The BLE flow for orientation mirrors the structure from Lab 5. Python sends `ORIENT_START` to reset the IMU yaw and begin the PID loop on Artemis, which auto-stops after a timeout. During the run the Artemis buffers every PID sample into fixed arrays. After Python sends `ORIENT_STOP` and then `GET_ORIENT_DATA`, the Artemis streams back tagged strings. Gains and the setpoint can be updated at any time mid-run without reflashing, using `SET_ORIENT_GAINS` and `SET_ORIENT_TARGET`.

The data format from Artemis is:

```
OPID|{yaw_tenths}|{error_tenths}|{motor_pwm}|{time_ms}
OPID_END|{count}
```

Yaw and error are transmitted as integers in tenths of a degree so that 905 represents 90.5°. Motor PWM is signed: positive means right-turn drive, negative means left-turn drive.

<details>
<summary>Arduino: GET_ORIENT_DATA packet format</summary>
<div markdown="1">

```cpp
case GET_ORIENT_DATA:
{
    for (int i = 0; i < orient_pos; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("OPID|");
        tx_estring_value.append((int)(orient_yaw_hist[i] * 10));
        tx_estring_value.append("|");
        tx_estring_value.append((int)(orient_err_hist[i] * 10));
        tx_estring_value.append("|");
        tx_estring_value.append((int)orient_motor_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)orient_t_hist[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        BLE.poll();
    }
    tx_estring_value.clear();
    tx_estring_value.append("OPID_END|");
    tx_estring_value.append(orient_pos);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    break;
}
```

</div>
</details>

<details>
<summary>Python: notification handler and parser</summary>
<div markdown="1">

```python
_orient_buf  = []
_orient_done = False

def _orient_notify_handler(uuid, bytearray_data):
    global _orient_buf, _orient_done
    line = ble.bytearray_to_string(bytearray_data).strip()
    _orient_buf.append(line)
    if line.startswith('OPID_END'):
        _orient_done = True

def parse_orient_data(buf):
    rows = []
    for line in buf:
        if line.startswith('OPID|'):
            parts = line.split('|')
            if len(parts) == 5:
                rows.append({
                    'yaw_deg':   int(parts[1]) / 10.0,
                    'error_deg': int(parts[2]) / 10.0,
                    'motor_pwm': int(parts[3]),
                    'time_ms':   int(parts[4]),
                })
    df = pd.DataFrame(rows)
    if len(df) > 0:
        df['time_s'] = (df['time_ms'] - df['time_ms'].min()) / 1000.0
    return df
```

</div>
</details>

The `run_orient_experiment()` helper calls `ORIENT_START`, optionally fires `SET_ORIENT_TARGET` mid-run to test setpoint changes, then calls `ORIENT_STOP` and `GET_ORIENT_DATA` and blocks until `OPID_END` arrives. This keeps BLE responsive the whole time since the Artemis is only logging and the gains or setpoint can be changed by any Python call while the loop is running.

## Gyro Integration and Sensor Considerations

I use the onboard DMP to get yaw directly, which avoids the accumulating drift that raw gyro integration causes. Without the DMP, integrating the raw gyro introduces bias error that grows without bound. The ICM-20948 gyroscope has a default full-scale range of ±250 °/s, which is sufficient for in-place turns at the speeds I drive. The DMP outputs a quaternion that I convert to a single yaw angle referenced to the initial heading at startup.

The PID loop ran at an average of 4.5 ms per iteration, roughly 220 Hz. That is well above the DMP output rate of about 100 Hz, so the loop has fresh yaw data on nearly every other iteration.

The error is clamped to [-180°, 180°] before entering the PID so the robot always takes the shortest path. Without this clamp, a 181° error and a -179° error would produce opposite motor commands even though the robot is almost at the same position.

## P Control

Starting KP: at 180° error I want close to maximum turn speed. The motor PWM is mapped from PID output into [110, 166] after a deadband, so the effective output range is [-166, 166]. KP = 166 / 180 ≈ 0.92 gives full turn speed at maximum error. I started at KP = 2.5, tuned empirically since the deadband mapping made the effective gain lower than the raw number.

With KP = 2.5 and target = 90°, the robot reached 90° within about 1.0 s and the final error settled to +0.1°. There was overshoot to about 93.7° before settling. The PID loop interval averaged 4.3 ms (234 Hz) for 966 samples over a 4.1 s run.

<img src='/images/mae4190/lab6/lab6_p_control.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab6/p_right_turn_90.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">P control turning to 90°. Overshoot visible before settling.</div>
</div>

## PD Control

The derivative term damps the motor output when the yaw is changing quickly, reducing overshoot. The derivative is computed on the yaw error directly. Since yaw is already an integral of angular velocity, the derivative of yaw error is angular velocity with sign. This is a meaningful signal because it tells the controller how fast the robot is rotating, so it can apply a counteracting torque before it overshoots.

I set KD = 0.05. The first attempt used KD = 0.4, but at 300 °/s the D term alone contributed 120 PWM, overpowering the P term at errors below 48°. That produced oscillation instead of damping. Dropping to 0.05 gave clean settling.

I tested at target = -180° to verify wrap-around logic. The robot correctly chose the left-turn direction, rotated through -180°/+180° cleanly, and stopped at 179.9° with a final error of +0.1°. The 982-sample run averaged 4.8 ms per loop (208 Hz).

<img src='/images/mae4190/lab6/lab6_pd_control.png' width='700'>

<div style="display:flex; gap:8px; flex-wrap:wrap; align-items:flex-start;">
  <div style="width:49%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab6/pd_right_turn_90.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">PD control at 90°.</div>
  </div>
  <div style="width:49%; text-align:center;">
    <video width='100%' controls><source src='/images/mae4190/lab6/pd_turn_180.mp4' type='video/mp4'></video>
    <div style="font-size:0.9em;">PD control turning to -180°, testing wrap-around.</div>
  </div>
</div>

The comparison below shows P vs PD at the same 90° target. PD reaches the setpoint faster and the motor output drops smoothly as the robot approaches rather than cutting sharply.

<img src='/images/mae4190/lab6/lab6_p_vs_pd.png' width='700'>

## Derivative Kick and Lowpass Filter

Derivative kick happens when the setpoint changes instantaneously mid-run. The error jumps by a large amount in one PID iteration, so the derivative term sees a spike and drives the motor to maximum speed for one step. The fix is a first-order lowpass filter on the derivative term:

```cpp
orient_dF = alpha * d_raw + (1.0f - alpha) * orient_dF;
```

With alpha = 1.0 there is no filtering and the spike passes through. With alpha = 0.002 the filter is aggressive and the derivative can only move by 0.2% of the raw value per step, blocking the kick entirely.

I ran two tests with the same gain and setpoint change sequence: hold at 90° for 3 s, then switch target to -90°. Without the filter the motor PWM spiked to ±200 at the moment of the setpoint switch. With alpha = 0.002 the motor output was smooth through the transition.

<img src='/images/mae4190/lab6/lab6_derivative_kick.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab6/derivative_kick.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">Derivative kick demo: without LPF the motor spikes at the setpoint change; with LPF α=0.002 the transition is smooth.</div>
</div>

<details>
<summary>Arduino: derivative LPF in orientation PID</summary>
<div markdown="1">

```cpp
float d_raw = (orient_error - orient_last_e) / ((float)dt / 1000.0f);
orient_dF = ORIENT_ALPHA * d_raw + (1.0f - ORIENT_ALPHA) * orient_dF;
orient_last_e = orient_error;

float output = orient_kp * orient_error
             + orient_ki * orient_I
             + orient_kd * orient_dF;
```

</div>
</details>

## PID Control

Adding the integral term removes steady-state error that friction causes. When the robot sits slightly off the setpoint and P+D are too small to move it, the integrator winds up over time until the combined output overcomes the deadband and the motors fire. I used KI = 0.05.

The PID run at 90° showed a final error of -1.0° compared to +0.1° for P-only. The integrator slightly overshot the target by 1°, meaning accumulation was still ongoing when the robot stopped. A tighter wind-up clamp would fix this. The loop averaged 4.7 ms (213 Hz) over 1015 samples.

<img src='/images/mae4190/lab6/lab6_pid_control.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab6/pid_right_turn_90.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">PID control to 90°.</div>
</div>

<details>
<summary>Arduino: full orientation PID computation</summary>
<div markdown="1">

```cpp
// Clamp error to [-180, 180] for shortest-path rotation
float orient_error = orient_target - orient_yaw;
while (orient_error >  180.0f) orient_error -= 360.0f;
while (orient_error < -180.0f) orient_error += 360.0f;

// Integral with wind-up clamp
orient_I += orient_error * ((float)dt / 1000.0f);
if (orient_I >  ORIENT_IMAX) orient_I =  ORIENT_IMAX;
if (orient_I < -ORIENT_IMAX) orient_I = -ORIENT_IMAX;

// Derivative with LPF
float d_raw = (orient_error - orient_last_e) / ((float)dt / 1000.0f);
orient_dF = ORIENT_ALPHA * d_raw + (1.0f - ORIENT_ALPHA) * orient_dF;
orient_last_e = orient_error;

float output = orient_kp * orient_error
             + orient_ki * orient_I
             + orient_kd * orient_dF;
output = constrain(output, -200.0f, 200.0f);

// Deadband mapping into [TURN_MIN=110, TURN_MAX=166]
if (fabsf(output) > 2.0f) {
    float mapped = 110.0f + (fabsf(output) / 200.0f) * 56.0f;
    int pwm = (int)constrain(mapped, 110.0f, 166.0f);
    if (output > 0) motorsTurnRight(pwm);
    else            motorsTurnLeft(pwm);
} else {
    motorsStop();
}
```

</div>
</details>

The three-way comparison plot shows that P and PD settle similarly fast, but PID removes the small residual offset caused by surface friction.

<img src='/images/mae4190/lab6/lab6_pid_comparison.png' width='700'>

| Controller | KP  | KI   | KD   | Final error | Avg loop |
|---|---|---|---|---|---|
| P   | 2.5 | 0    | 0    | +0.1° | 4.3 ms |
| PD  | 2.5 | 0    | 0.05 | +0.6° | 4.4 ms |
| PID | 2.5 | 0.05 | 0.05 | -1.0° | 4.7 ms |

## Setpoint Change Mid-Run

To verify real-time setpoint updates, I started at 90°, let the robot converge, then sent `SET_ORIENT_TARGET -90` over BLE at t = 4 s while the PID loop was still running. The robot reached 89.9° at the end of the first phase, then immediately started turning toward -90° after the command was received.

<img src='/images/mae4190/lab6/lab6_pid_control_with_setpoint_change.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab6/pid_setpoint_change.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">PID control with setpoint change from 90° to -90° at t=4 s via BLE.</div>
</div>

<details>
<summary>Arduino: SET_ORIENT_TARGET command handler</summary>
<div markdown="1">

```cpp
case SET_ORIENT_TARGET:
{
    float new_target;
    success = robot_cmd.get_next_value(new_target);
    if (success) {
        orient_target = new_target;
        tx_estring_value.clear();
        tx_estring_value.append("ORIENT_TARGET|");
        tx_estring_value.append(orient_target);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
}
```

</div>
</details>

The setpoint variable is a global the PID loop reads on every iteration, so writing it from a BLE command handler takes effect immediately on the next loop cycle. No synchronization primitives are needed because the Artemis is single-threaded and BLE events are only processed when `BLE.poll()` is called explicitly between data transmissions.

## Wind-up Protection

The integrator is clamped to ±`ORIENT_IMAX` to prevent runaway accumulation. On a slippery floor the robot may spin past the setpoint without the integrator contributing much, but on a high-friction surface the integrator winds up quickly. Without the clamp, switching to a new setpoint on a rough surface produced about 15° of transient overshoot because the accumulated integral from the previous hold was still pushing in the old direction. With the clamp at ±50, overshoot on surface change dropped to under 5°.

<img src='/images/mae4190/lab6/lab6_pid_uneven.png' width='700'>

Meet with my cat Mulberry! 🐱

<img src='/images/mae4190/cats/cat1.png' width='400'>
<img src='/images/mae4190/cats/cat2.png' width='400'>

[Back to MAE 4190](/mae4190/)
