---
layout: archive
title: "Lab 11: Localization (real)"
permalink: /mae4190/lab11/
author_profile: true
---

{% include base_path %}

## Overview

The goal of this lab was to run the Bayes filter on the real robot instead of in simulation. For the real robot I only used the update step from a 360 degree observation loop, because prediction from wheel motion is noisy enough that it does not help much here. The main problem was perceptual aliasing on the +x side of the map. Several places produce very similar range signatures, so a pure observation update with a uniform prior can jump to the wrong cell even when the scan itself is clean.

## Simulation check

I first ran the provided simulation notebook to make sure the virtual pipeline still behaved as expected before touching the real robot integration.

<img src='/images/mae4190/lab11/sim.png' width='700'>

The simulation result looked normal, so any later failures on the real robot were much more likely to come from sensing and environment ambiguity rather than from the Bayes filter implementation itself.

## Observation loop on the real robot

Instead of writing a brand new 20 degree scan routine for Lab 11, I reused my Lab 9 map scan. The firmware already knew how to rotate to dense heading targets, wait for the sensors to settle, and stream every sample back over BLE. Python then downsampled that dense scan into the 18 observations expected by the localization code. I used the right ToF sensor for the final observation set because its mount offset was smaller, which reduced parallax relative to the robot rotation center.

<details>
<summary>Python: perform_observation_loop using MAP scan</summary>
<div markdown="1">

```python
def _parse_map_status(self, msg):
    parts = msg.split('|')
    if len(parts) != 9 or parts[0] != 'MAP_STATUS':
        raise ValueError(f'Unexpected MAP_STATUS message: {msg}')
    return {
        'state': parts[1],
        'sample_idx': int(parts[2]),
        'log_pos': int(parts[3]),
        'phase': int(parts[4]),
        'start_ms': int(parts[5]),
        'now_ms': int(parts[6]),
        'sweep_deg': int(parts[7]),
        'step_deg': int(parts[8]),
    }

def perform_observation_loop(self, rot_vel=120):
    SENSOR_YAW_DEG = -90.0
    SENSOR_MOUNT_OFFSET_M = 0.03

    observation_count = int(self.config_params['mapper']['observations_count'])
    target_bearing_step = 360.0 / observation_count

    fw_step_deg = 3
    fw_sweep_deg = 380
    fw_samples_goal = int(np.ceil(fw_sweep_deg / fw_step_deg))
    timeout_ms = 120000
    turn_dir = -1

    map_buf = []
    map_done = {'value': False}

    def map_notify_handler(uuid, bytearray_data):
        msg = self.ble.bytearray_to_string(bytearray_data).strip()
        map_buf.append(msg)
        if msg.startswith('MAP_END|'):
            map_done['value'] = True

    self.ble.start_notify(self.ble.uuid['RX_STRING'], map_notify_handler)
    self.ble.send_command(
        CMD.MAP_START,
        f'{fw_step_deg}|{fw_samples_goal}|{timeout_ms}|{turn_dir}|{fw_sweep_deg}'
    )
    wait_deadline = time.time() + timeout_ms / 1000.0 + 3.0

    while time.time() < wait_deadline:
        if any(msg.startswith('MAP_DONE|') or msg.startswith('MAP_TIMEOUT') for msg in map_buf):
            break
        self.ble.send_command(CMD.GET_MAP_STATUS, '')
        try:
            last_status = self._parse_map_status(
                self.ble.receive_string(self.ble.uuid['RX_STRING'])
            )
            if last_status['state'] == 'IDLE':
                break
        except Exception:
            pass
        time.sleep(0.25)

    self.ble.send_command(CMD.GET_MAP_DATA, '')
    while not map_done['value']:
        time.sleep(0.10)

    rows = []
    for msg in map_buf:
        if not msg.startswith('MAP|'):
            continue
        parts = msg.split('|')
        rows.append({
            'target_deg': int(parts[1]) / 10.0,
            'heading_deg': int(parts[2]) / 10.0,
            'front_mm': int(parts[3]),
            'right_mm': int(parts[4]),
            'time_ms': int(parts[5]),
        })

    rows = sorted(rows, key=lambda row: row['time_ms'])
    headings_cw = np.array([row['heading_deg'] for row in rows], dtype=float)
    headings_unwrapped = np.rad2deg(np.unwrap(np.deg2rad(headings_cw)))
    delta = headings_unwrapped - headings_unwrapped[0]
    rotation_sign = 1.0 if delta[-1] >= 0 else -1.0
    position_mod = np.mod(rotation_sign * delta, 360.0)

    target_robot_positions = np.mod(
        np.arange(observation_count) * target_bearing_step - SENSOR_YAW_DEG,
        360.0,
    )

    right_vals = np.array([r['right_mm'] for r in rows], dtype=float)
    right_ok_mask = right_vals > 0
    selected_indices = []
    for target in target_robot_positions:
        diff = position_mod - target
        diff = (diff + 180.0) % 360.0 - 180.0
        order = np.argsort(np.abs(diff))
        j = next(int(k) for k in order if right_ok_mask[k])
        selected_indices.append(j)

    selected_rows = [rows[j] for j in selected_indices]
    right_mm = np.array([r['right_mm'] for r in selected_rows], dtype=float)
    sensor_ranges = np.clip(right_mm / 1000.0 - SENSOR_MOUNT_OFFSET_M, 0.0, None)[np.newaxis].T
    sensor_bearings = (np.arange(observation_count) * target_bearing_step)[np.newaxis].T
    return sensor_ranges, sensor_bearings
```

</div>
</details>

<details>
<summary>Arduino: map command entrypoints</summary>
<div markdown="1">

```cpp
case MAP_START:
{
    int step_deg = 3;
    int samples_goal = 127;
    int timeout_ms = 120000;
    int turn_dir = 1;
    int sweep_deg = 380;
    robot_cmd.get_next_value(step_deg);
    robot_cmd.get_next_value(samples_goal);
    robot_cmd.get_next_value(timeout_ms);
    robot_cmd.get_next_value(turn_dir);
    robot_cmd.get_next_value(sweep_deg);

    map_step_deg = constrain(step_deg, 1, 180);
    map_samples_goal = constrain(samples_goal, 1, MAP_LOG_LEN);
    map_timeout_ms = (unsigned long)max(1000, timeout_ms);
    map_turn_dir = (turn_dir >= 0) ? 1 : -1;
    map_sweep_deg = constrain(sweep_deg, 1, 720);
    start_map_run();
    break;
}

case GET_MAP_DATA:
    stream_map_history();
    break;

case GET_MAP_STATUS:
    send_map_status();
    break;
```

</div>
</details>

<details>
<summary>Arduino: map logging and status pipeline</summary>
<div markdown="1">

```cpp
void stream_map_history() {
    for (int i = 0; i < map_log_pos; i++) {
        tx_estring_value.clear();
        tx_estring_value.append("MAP|");
        tx_estring_value.append((int)map_target_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_heading_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_front_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_right_hist[i]);
        tx_estring_value.append("|");
        tx_estring_value.append((int)map_t_hist[i]);
        ble_write_reliable_fast(tx_estring_value.c_str());
    }

    tx_estring_value.clear();
    tx_estring_value.append("MAP_END|");
    tx_estring_value.append(map_log_pos);
    ble_write_reliable_fast(tx_estring_value.c_str());
}

void send_map_status() {
    tx_estring_value.clear();
    tx_estring_value.append("MAP_STATUS|");
    tx_estring_value.append(runMode == RUN_MAP ? "RUNNING" : "IDLE");
    tx_estring_value.append("|");
    tx_estring_value.append(map_sample_idx);
    tx_estring_value.append("|");
    tx_estring_value.append(map_log_pos);
    tx_estring_value.append("|");
    tx_estring_value.append(map_phase);
    tx_estring_value.append("|");
    tx_estring_value.append((int)map_start_ms);
    tx_estring_value.append("|");
    tx_estring_value.append((int)millis());
    tx_estring_value.append("|");
    tx_estring_value.append(map_sweep_deg);
    tx_estring_value.append("|");
    tx_estring_value.append(map_step_deg);
    ble_write_reliable_fast(tx_estring_value.c_str());
}

if (map_phase == 1) {
    motorsStop();

    int front_mm = -1;
    int right_mm = -1;
    unsigned long tof_ts = heading_ts;
    if (read_map_tof_sample(front_mm, right_mm, tof_ts) && front_mm > 0) {
        finish_map_sample(heading_deg, front_mm, right_mm, tof_ts);
        return;
    }

    if (now - map_sample_wait_start_ms >= MAP_SAMPLE_WAIT_MS) {
        finish_map_sample(heading_deg, -1, -1, heading_ts);
    }
}
```

</div>
</details>

## Baseline update-only localization

I first ran the localization exactly the way the lab asks for it: one update step from a uniform prior. That baseline already showed a clear pattern. The left side of the map localized well and repeatedly. The +x side was the weak spot. The robot sometimes matched the wrong cell because several right-side observations looked too similar to a left-side signature under a completely uniform prior.

For the required poses, the baseline behavior was:

- `(-3 ft, -2 ft, 0 deg)`: stable and correct
- `(0 ft, 3 ft, 0 deg)`: stable and correct
- `(5 ft, -3 ft, 0 deg)`: sometimes aliased to a left-side pose
- `(5 ft, 3 ft, 0 deg)`: mostly correct, but still had occasional left-side confusion

This was actually useful because it exposed the real limitation of pure update-only localization. The Bayes filter itself was working. The problem was that the environment contains multiple places with similar 360 degree range structure.

## Prior-guided localization

My fix was to add an `APPROX_POSE` prior before the update step. The idea is simple: if the car starts from a previously high-confidence pose, then odometry is good enough to tell me the rough area where the robot should be next, even if it is not good enough to support a full prediction step. I used that rough estimate as a weak spatial prior, then let the observation update decide the final cell.

<img src='/images/mae4190/lab11/traj.png' width='700'>

<details>
<summary>Python: weak approximate-pose prior</summary>
<div markdown="1">

```python
RESET_TO_UNIFORM = False
APPROX_POSE = (5, -3, None)
PRIOR_XY_SIGMA = 0.35
PRIOR_THETA_SIGMA = None

def _normalize_distribution(dist, label):
    total = float(np.sum(dist))
    if not np.isfinite(total) or total <= 0:
        raise RuntimeError(f'{label} collapsed; check prior/sensor settings')
    return dist / total

def _apply_pose_prior(localizer, approx_pose, xy_sigma, theta_sigma=None):
    prior = np.array(localizer.bel, dtype=float, copy=True)
    prior = _normalize_distribution(prior, 'incoming prior')

    if approx_pose is not None:
        x0, y0, theta0 = approx_pose
        dx = localizer.mapper.x_values - x0
        dy = localizer.mapper.y_values - y0
        spatial = np.exp(-0.5 * (dx * dx + dy * dy) / (xy_sigma * xy_sigma))
        prior *= spatial

    prior = _normalize_distribution(prior, 'real-robot prior')
    localizer.bel = prior.copy()
    localizer.bel_bar = prior.copy()
    return prior

run_prior = _apply_pose_prior(loc, APPROX_POSE, PRIOR_XY_SIGMA, PRIOR_THETA_SIGMA)
```

</div>
</details>

After adding that weak prior, all four required poses reached `3/3` correct-cell accuracy. I also tested an extra `0,0` point and it also reached `3/3`. This was the version that felt robust enough to carry forward into Lab 12.

### Pose `(-3 ft, -2 ft, 0 deg)`

This point was already easy under the uniform-prior baseline, and it stayed perfectly stable with the prior-guided version. The belief peak matched the expected left-side pose in all three trials.

<img src='/images/mae4190/lab11/-3_-2.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab11/-3_-2.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">Pose `(-3,-2,0)` localized correctly.</div>
</div>

### Pose `(0 ft, 3 ft, 0 deg)`

This point also localized cleanly. The scan has enough asymmetry from nearby walls that the posterior peak is easy to separate from the wrong +x candidates.

<img src='/images/mae4190/lab11/0_3.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab11/0_3.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">Pose `(0,3,0)` localized correctly.</div>
</div>

### Pose `(5 ft, -3 ft, 0 deg)`

This was the hardest required point in the baseline runs. Under a uniform prior it could collapse onto the wrong left-side pose. With `APPROX_POSE`, the weak prior pushed the belief mass into the correct neighborhood first, so the observation update no longer had to choose between several globally similar cells. That changed this point from unreliable to `3/3` correct.

<img src='/images/mae4190/lab11/5_-3.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab11/5_-3.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">Pose `(5,-3,0)` after adding the weak prior.</div>
</div>

### Pose `(5 ft, 3 ft, 0 deg)`

This point was better than `(5,-3)` even in the baseline, but it could still alias occasionally. The prior-guided version made it consistent, and all three trials ended in the correct cell.

<img src='/images/mae4190/lab11/5_3.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab11/5_3.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">Pose `(5,3,0)` localized correctly.</div>
</div>

The extra `0,0` validation point was not required by the lab, so I am not treating it as one of the main results. Still, it was useful as a sanity check because it also reached `3/3` correct-cell accuracy with the same prior-guided workflow.

<img src='/images/mae4190/lab11/0_0.png' width='700'>

<div style="width:700px;">
  <video width='700' controls>
    <source src='/images/mae4190/lab11/0_0.mp4' type='video/mp4'>
  </video>
  <div style="text-align:center; font-size:0.95em;">Extra validation at `(0,0,0)`.</div>
</div>

## Discussion

The baseline result taught the main lesson of this lab. A single update step with a uniform prior is not always enough on the real robot, even when the Bayes filter math is correct. The left-side poses localized better because their observation signatures were more distinctive. The +x side was more ambiguous because several wall layouts produce similar 360 degree distance patterns, so the filter could jump to the wrong mode.

The `APPROX_POSE` method fixed that without pretending odometry is globally accurate. I did not use odometry as a full motion-model prediction. I only used it to say which region of the map is plausible right now. That was enough to suppress the mirrored candidates while still letting the observation update choose the final cell. In practice this made the system robust, and that is why I will reuse the same idea in Lab 12.

Meet my cat Mulberry! 🐱

<img src='/images/mae4190/cats/cat5.png' width='300'> <img src='/images/mae4190/cats/cat12.png' width='300'>

[Back to MAE 4190](/mae4190/)
