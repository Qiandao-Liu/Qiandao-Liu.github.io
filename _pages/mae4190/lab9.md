---
layout: archive
title: "Lab 9: Mapping"
permalink: /mae4190/lab9/
author_profile: true
---

{% include base_path %}

## Goal

The goal of this lab was to build a static map of the room from several marked robot positions, transform every ToF reading into the room frame, and then convert the point cloud into a line based map for later localization and planning. I used orientation control, not open loop control. The robot turned in small angular steps, stopped, measured distance, and then turned to the next heading.

I scanned the four required points `(5, -3)`, `(-3, -2)`, `(0, 3)`, and `(5, 3)`. I also added `(0, 0)` because the extra center view made the merged map easier to clean up.

## Sensing And Control

I chose the right ToF sensor instead of the front sensor because it is closer to the robot center. That reduces the position error caused by any small off axis rotation. It also makes the measured angle more faithful during a turn. The sensor placement is shown below.

<img src="/images/mae4190/lab9/all_parts_distribution_diagram.png" width="700">
<div style="text-align:center; font-size:0.95em;">Mechanical layout of the robot. The right ToF sensor is closer to the rotation center than the front ToF sensor.</div>

For control, I used a `P only` orientation controller with a `3 degree` target spacing and a `380 degree` sweep. I intentionally overswept by `20 degrees` because static friction sometimes caused the last part of a `360 degree` turn to come up short. Each pass therefore targeted `127` headings. I ran both clockwise and counterclockwise passes at each location to check repeatability. Across the final `10` passes, the robot collected `1270` right sensor samples and all `1270` were valid. The mean absolute heading error averaged `4.51 degrees` across runs, and the worst case heading error was `10.2 degrees`.

One thing I had to correct during post processing was the room frame start heading for each scan. I first assumed every scan started with the same room heading, but the actual robot placement differed by `90 degrees` at several locations. I fixed that with rigid per scan `theta` corrections in the notebook. I did not scale any scan.

The figure below shows the direct relationship between angle and measured distance for all five locations. Each subplot overlays the clockwise and counterclockwise passes. The top row in each scan figure is the most useful one for mapping because it shows how the measured heading lines up with the right ToF distance profile. The lower plots show that the measured heading stayed close to the commanded heading, which is why I trusted IMU heading instead of assuming perfectly uniform angular spacing.

<img src="/images/mae4190/lab9/lab9_angle_relationship_overview.png" width="700">
<div style="text-align:center; font-size:0.95em;">Angle to distance relationship for all five scan locations. Blue is clockwise and orange is counterclockwise.</div>

## Code

<details>
<summary>Arduino: step and stop mapping with right ToF only</summary>
<div markdown="1">

```cpp
bool read_map_tof_sample(int &front_mm, int &right_mm, unsigned long &ts_ms) {
    float right_dist = 0.0f;
    if (!read_right_tof_sample(right_dist, ts_ms)) return false;

    front_mm = -1;
    right_mm = (int)right_dist;
    return true;
}

void handle_map() {
    if (runMode != RUN_MAP) return;

    unsigned long now = millis();
    if (now - map_start_ms >= map_timeout_ms) {
        motorsStop();
        runMode = RUN_IDLE;
        tx_characteristic_string.writeValue("MAP_TIMEOUT");
        return;
    }

    float heading_deg = 0.0f;
    float gyro_dps = imu_last_gyr_z;
    unsigned long heading_ts = now;
    if (!update_drift_heading_state(heading_deg, gyro_dps, heading_ts)) return;

    if (map_phase == 0) {
        float e = 0.0f;
        int pwm = 0;
        if (!step_orient_pid_with_heading(heading_deg, e, pwm, heading_ts, false)) return;

        bool settled = fabsf(e) <= MAP_DONE_BAND_DEG && fabsf(gyro_dps) <= MAP_DONE_GYRO_DPS;
        map_done_count = settled ? (map_done_count + 1) : 0;

        if (map_done_count >= MAP_DONE_COUNT) {
            motorsStop();
            map_sample_wait_start_ms = heading_ts;
            map_phase = 1;
        }
        return;
    }

    if (map_phase == 1) {
        int front_mm = -1;
        int right_mm = -1;
        unsigned long tof_ts = heading_ts;
        if (read_map_tof_sample(front_mm, right_mm, tof_ts)) {
            finish_map_sample(heading_deg, front_mm, right_mm, tof_ts);
            return;
        }
    }
}
```

</div>
</details>

<details>
<summary>Python: scan command and room frame transform</summary>
<div markdown="1">

```python
SWEEP_DEG = 380
STEP_DEG = 3
SAMPLES_GOAL = math.ceil(SWEEP_DEG / STEP_DEG)
ORIENT_KP = 2.0
ORIENT_KI = 0.0
ORIENT_KD = 0.0

def run_map_scan(scan_name, step_deg=STEP_DEG, samples_goal=SAMPLES_GOAL,
                 timeout_ms=MAP_TIMEOUT_MS, turn_dir=TURN_DIR_CW,
                 pass_name='pass', sweep_deg=SWEEP_DEG):
    ble.send_command(CMD.MAP_START,
                     f'{step_deg}|{samples_goal}|{timeout_ms}|{turn_dir}|{sweep_deg}')
    ...

def sensor_hits_room(df, pose, sensor='right', run_name='scan', scan_key=None, pass_name='pass'):
    valid = df['right_valid']
    ranges_ft = df.loc[valid, 'right_ft'].to_numpy()
    headings_deg = df.loc[valid, 'heading_deg'].to_numpy()
    room_origin = np.array([pose['x_ft'], pose['y_ft']])

    rows = []
    for heading_deg, rng_ft in zip(headings_deg, ranges_ft):
        body_to_room = rot2(pose['theta_deg'] + heading_deg)
        ray_body = np.array([np.cos(np.deg2rad(RIGHT_SENSOR_YAW_DEG)),
                             np.sin(np.deg2rad(RIGHT_SENSOR_YAW_DEG))]) * rng_ft
        hit_room = room_origin + body_to_room @ (RIGHT_SENSOR_OFFSET_FT + ray_body)
        rows.append({'scan_key': scan_key, 'x_ft': hit_room[0], 'y_ft': hit_room[1]})
    return pd.DataFrame(rows)
```

</div>
</details>

## Individual Scans

The five videos below show one measurement run at each scan location. After both passes at each location were merged, the clean point counts were `241` for `(5, -3)`, `220` for `(0, 0)`, `208` for `(-3, -2)`, `194` for `(0, 3)`, and `229` for `(5, 3)`.

<div style="display:flex; gap:12px; flex-wrap:wrap; align-items:flex-start;">
  <div style="width:48%; text-align:center;">
    <img src="/images/mae4190/lab9/scan_5_-3_pcd.png" width="100%">
    <video width="100%" controls>
      <source src="/images/mae4190/lab9/IMG_3130.mp4" type="video/mp4">
    </video>
    <div style="font-size:0.95em;">Scan at `(5, -3)`.</div>
  </div>
  <div style="width:48%; text-align:center;">
    <img src="/images/mae4190/lab9/scan_0_0_pcd.png" width="100%">
    <video width="100%" controls>
      <source src="/images/mae4190/lab9/IMG_3131.mp4" type="video/mp4">
    </video>
    <div style="font-size:0.95em;">Scan at `(0, 0)`.</div>
  </div>
  <div style="width:48%; text-align:center;">
    <img src="/images/mae4190/lab9/scan_-3_-2_pcd.png" width="100%">
    <video width="100%" controls>
      <source src="/images/mae4190/lab9/IMG_3132.mp4" type="video/mp4">
    </video>
    <div style="font-size:0.95em;">Scan at `(-3, -2)`.</div>
  </div>
  <div style="width:48%; text-align:center;">
    <img src="/images/mae4190/lab9/scan_0_3_pcd.png" width="100%">
    <video width="100%" controls>
      <source src="/images/mae4190/lab9/IMG_3133.mp4" type="video/mp4">
    </video>
    <div style="font-size:0.95em;">Scan at `(0, 3)`.</div>
  </div>
  <div style="width:48%; text-align:center;">
    <img src="/images/mae4190/lab9/scan_5_3_pcd.png" width="100%">
    <video width="100%" controls>
      <source src="/images/mae4190/lab9/IMG_3134.mp4" type="video/mp4">
    </video>
    <div style="font-size:0.95em;">Scan at `(5, 3)`.</div>
  </div>
</div>

The single scan plots were my sanity check before merging. They matched the expected nearby wall directions, which told me the angle logging and sensor transform were basically correct.

<img src="/images/mae4190/lab9/scan_5_-3_angle_relationship.png" width="700">
<div style="text-align:center; font-size:0.95em;">Representative angle tracking and angle to distance plot for the `(5, -3)` scan.</div>

## Merged Map

I merged all scans into the room frame with rigid transforms only. First I converted each right sensor hit into room coordinates using the robot position, the corrected room heading, and the fixed right sensor mounting angle. Then I cleaned the cloud with two simple filters: a distance range filter from `0.15 ft` to `12 ft`, and a per scan radius filter that kept only points within `6 ft` of the scan origin. This removed most of the far grazing angle returns without deleting the useful walls near each robot location.

The raw merged cloud is shown below. It contains all `1270` valid measurements from the `10` runs.

<img src="/images/mae4190/lab9/lab9_global_map_direction_fixed_raw.png" width="700">
<div style="text-align:center; font-size:0.95em;">All transformed right ToF hits before cleanup.</div>

After cleanup, `1092` points remained. This version is much easier to fit with line segments.

<img src="/images/mae4190/lab9/lab9_global_map_direction_fixed_clean.png" width="700">
<div style="text-align:center; font-size:0.95em;">Merged map after range filtering, scan radius filtering, and rigid per scan heading correction.</div>

## Line Map And Discussion

I manually fit line segments to the cleaned point cloud and exported the line endpoints for the next lab. In the plot below, the black line segments are the walls and obstacles estimated from the point cloud, and the green line segments are the actual room boundaries and obstacles.

<img src="/images/mae4190/lab9/lab9_global_map_direction_fixed_clean_wall.png" width="700">
<div style="text-align:center; font-size:0.95em;">Black lines are the map estimated from the point cloud. Green lines are the real walls and obstacles.</div>

The outside walls came out very well. Those boundaries are the most reliable part of the map because they were seen from several locations and at more favorable angles. The middle wall segments and obstacles are less accurate. The main failure mode is that the point cloud usually makes obstacles look a little larger than they really are. That is still acceptable for path planning because it is conservative. The robot may choose a slightly longer path, but it is less likely to cut too close and collide. I think the remaining error comes from three sources: heading error during each turn, small placement error when moving the robot between marks, and ToF bias when the beam hits surfaces at an oblique angle.

Meet my cat Mulberry! 🐱

<img src="/images/mae4190/cats/cat12.png" width="400">
<img src="/images/mae4190/cats/cat11.png" width="400">

[Back to MAE 4190](/mae4190/)
