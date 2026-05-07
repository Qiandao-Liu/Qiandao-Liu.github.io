---
layout: archive
title: "Lab 12: Planning and Execution"
permalink: /mae4190/lab12/
author_profile: true
---

{% include base_path %}

## ECE Robotics Day 🦾

The robot also performed at ECE Robotics Day before lab 12 was due, doing the same drift stunt from Lab 8.

<video width="700" controls>
  <source src='/images/mae4190/lab12/ECE_Robotics_Day_drift_show.mp4' type='video/mp4'>
</video>

## Strategy

<div style="max-width:560px; margin:1em auto; text-align:center; font-size:0.88em; line-height:1.7;">
  <div style="display:inline-block; background:#d0e8ff; border:1px solid #6aabcc; border-radius:6px; padding:7px 20px;">
    Start: (-4, -3, 0°)
  </div>
  <div style="font-size:1.3em; color:#666; line-height:1.3;">↓</div>
  <div style="background:#d8f0d8; border:1px solid #5a9060; border-radius:6px; padding:8px 16px; text-align:left;">
    Initial Localization<br>
    <span style="color:#555;">360° ToF scan (onboard) → Bayes filter update (offboard) → confirm start pose</span>
  </div>
  <div style="font-size:1.3em; color:#666; line-height:1.3;">↓</div>
  <div style="background:#fff8dc; border:1px solid #c8a000; border-radius:6px; padding:8px 16px; text-align:left;">
    Open-Loop Phase — Segments 1, 2, 3<br>
    <span style="color:#555;">Gyro PID align heading → timed forward drive → front ToF safety</span>
  </div>
  <div style="font-size:1.3em; color:#666; line-height:1.3;">↓</div>
  <div style="display:flex; gap:10px; text-align:left;">
    <div style="flex:3; background:#ffe8e8; border:1px solid #c05050; border-radius:6px; padding:8px 14px;">
      Wall-Follow Phase — Segments 4, 6, 7, 8<br>
      <span style="color:#555;">Right ToF P-steer → front ToF PID speed/stop → all decisions onboard</span>
    </div>
    <div style="flex:2; background:#f3eaff; border:1px solid #9060aa; border-radius:6px; padding:8px 14px;">
      Segment 5 — open-loop step<br>
      <span style="color:#555;">Short move forward, but keep using rigth sensor follow the wall</span>
    </div>
  </div>
  <div style="font-size:1.3em; color:#666; line-height:1.3;">↓</div>
  <div style="display:inline-block; background:#d0e8ff; border:1px solid #6aabcc; border-radius:6px; padding:7px 20px;">
    End: (0, 0)
  </div>
</div>

The task is to navigate through 9 fixed waypoints on a known map. I did not implement global path planning. All the waypoints are given in advance and nothing in the environment moves, so there is nothing to plan online. Running the laptop as a real-time planner would also be slow since BLE communication adds too much latency for closed-loop corrections to be useful.

The path splits naturally into two phases. Segments 1 through 3 travel through open space at diagonal and lateral angles where wall references are not reliable, so I used gyroscope-based orientation control with timed forward drive. Segments 4 through 8 run along or close to the surrounding walls, so I switched to a wall-following controller using both the front and right ToF sensors. All motion decisions run onboard on the Artemis. The laptop only triggers segments and handles the Bayes filter computation.

## Open-Loop Phase

The first three segments move from (-4,-3) to (-2,-1), then to (1,-1), then down to (2,-3). For each segment the robot first aligns to the correct heading using a gyroscope PID loop, then drives forward. The heading for each segment is derived geometrically from the waypoint coordinates. The front ToF sensor watches for unexpected obstacles throughout and stops the drive early if the robot gets too close to a wall.

IMU drift over these short distances is small enough that the position error at waypoint 3 is acceptable as a handoff into wall-following. The geometry of waypoints 1 through 3 and the surrounding walls makes it difficult to use a consistent right-sensor reference anyway, so timed drive is the pragmatic choice here.

<details>
<summary>Python: open-loop segment execution</summary>
<div markdown="1">

```python
def orient_to_absolute_heading(target_deg, current_heading_deg, run_s=ORIENT_RUN_S, skip_band_deg=3.0):
    delta_deg = _wrap_deg(target_deg - current_heading_deg)
    if abs(delta_deg) <= skip_band_deg:
        return target_deg, 0.0
    ble.send_command(CMD.SET_ORIENT_TARGET, f'{delta_deg:.2f}')
    ble.send_command(CMD.ORIENT_START, '')
    time.sleep(run_s)
    ble.send_command(CMD.ORIENT_STOP, '')
    time.sleep(0.35)
    return target_deg, delta_deg

def drive_open_loop(pwm, duration_ms):
    ble.send_command(CMD.MOTOR_FORWARD, f'{int(pwm)}|{int(duration_ms)}')
    time.sleep(duration_ms / 1000.0)
    ble.send_command(CMD.MOTOR_BACKWARD, f'{OPEN_LOOP_BRAKE_PWM}|{OPEN_LOOP_BRAKE_MS}')
    time.sleep(OPEN_LOOP_BRAKE_MS / 1000.0 + 0.05)
    ble.send_command(CMD.MOTOR_STOP, '')
    time.sleep(OPEN_LOOP_SETTLE_S)

for seg in EARLY_SEGMENTS:
    target_heading = seg['control_heading_deg']  # computed from waypoint geometry
    new_heading, _ = orient_to_absolute_heading(target_heading, estimated_heading_deg)
    estimated_heading_deg = new_heading
    drive_open_loop(seg['base_pwm'], seg['duration_ms'])
```

</div>
</details>

<details>
<summary>Arduino: timed motor command and auto-stop</summary>
<div markdown="1">

```cpp
case MOTOR_FORWARD:
{
    int speed, duration;
    if (!robot_cmd.get_next_value(speed)) break;
    if (!robot_cmd.get_next_value(duration)) duration = 0;
    set_manual_motor(MOT_FWD, speed, duration, "MOTOR_FWD");
    break;
}

void handle_motors() {
    if (motorTimed && millis() >= motorEndTime) {
        motorsStop();
        motorMode = MOT_IDLE;
        motorTimed = false;
    }
}
```

</div>
</details>

## Initial Localization

I ran one Bayes filter update at the very start to verify the robot's actual placement at (-4,-3). Hand-placement is never exact. The filter applied a weak spatial Gaussian prior centered on the expected starting pose, then updated from a full 360-degree scan.

The update converged at a belief probability of 0.9986 to the grid cell at (-4.0 ft, -3.0 ft) with an estimated heading of -10 degrees. The position match was essentially exact. The small heading offset was noted and factored into the first turn command.

<details>
<summary>Python: Bayes filter update with weak pose prior</summary>
<div markdown="1">

```python
def _apply_pose_prior(localizer, approx_pose, xy_sigma):
    prior = np.array(localizer.bel, dtype=float, copy=True)
    prior /= np.sum(prior)
    x0, y0, _ = approx_pose
    dx = localizer.mapper.x_values - x0
    dy = localizer.mapper.y_values - y0
    prior *= np.exp(-0.5 * (dx**2 + dy**2) / xy_sigma**2)
    prior /= np.sum(prior)
    localizer.bel = prior.copy()
    localizer.bel_bar = prior.copy()

def run_checkpoint_localization(approx_pose, note=''):
    loc.init_grid_beliefs()
    _apply_pose_prior(loc, approx_pose, PRIOR_XY_SIGMA)
    loc.get_observation_data()   # triggers 360° scan over BLE
    loc.update_step()
    best_idx = np.unravel_index(int(np.argmax(loc.bel)), loc.bel.shape)
    best_prob = float(loc.bel[best_idx])
    best_pose = mapper.from_map(*best_idx)
    print(f'[{note}] best={best_pose}  prob={best_prob:.4f}')
    return {'best_pose': best_pose, 'best_prob': best_prob}
```

</div>
</details>

<div style="display:flex; gap:16px; align-items:flex-start; flex-wrap:wrap;">
  <div style="flex:1; min-width:300px; text-align:center;">
    <img src='/images/mae4190/lab12/first_localization_pos.png' width='100%'>
    <div style="font-size:0.9em; color:#555;">Initial localization at (-4,-3). Prob = 0.9986, estimated heading -10°.</div>
  </div>
  <div style="flex:1; min-width:300px; text-align:center;">
    <img src='/images/mae4190/lab12/second_localization_pos.png' width='100%'>
    <div style="font-size:0.9em; color:#555;">Second localization checkpoint after segment 3. Dropped in final run since wall PID corrects faster.</div>
  </div>
</div>

I originally planned a second localization checkpoint after segment 3, before entering the wall-following phase, as an extra correction step. In the final run I removed it. The wall-following controller corrects lateral offset within the first meter of each wall segment, so any drift from the open-loop phase gets absorbed quickly. The scan takes about 45 seconds over BLE and the benefit was not worth the time cost.

## Wall-Following Phase

Professor Helbling mentioned in lecture that a robot can use its right-side sensor to track a wall. Segments 4, 5, 6, 7, 8 all travel along walls, so I built the controller around exactly that idea.

The right ToF sensor feeds a P controller that steers left or right to hold a target standoff from the right wall. The front ToF sensor feeds a separate KF-PID controller that modulates forward speed and triggers a stop when the distance ahead drops to the expected endpoint distance. Both controllers run together in the onboard loop so the robot handles small wall irregularities without any laptop involvement. Segment 5 between (5,-3) and (5,-2) is a short one-foot open-loop step, so just let robot forward and stop quick while let right sensor follow the wall.

<details>
<summary>Arduino: wall-follow control loop</summary>
<div markdown="1">

```cpp
void handle_wall_follow() {
    if (runMode != RUN_WALL_FOLLOW) return;

    unsigned long now = millis();
    if (now - wall_start_ms >= wall_timeout_ms) {
        finish_wall_follow_run(3, true);  // TIMEOUT
        return;
    }

    float front_dist = 0.0f, right_dist = 0.0f;
    unsigned long front_ts = now, right_ts = now;
    bool got_front = read_front_tof_sample(front_dist, front_ts);
    bool got_right = read_right_tof_sample(right_dist, right_ts);

    if (got_front) {
        if (front_dist <= wall_front_safety_mm) { finish_wall_follow_run(2, true); return; }
        if (front_dist <= wall_front_stop_mm)   { finish_wall_follow_run(1, true); return; }
    }

    int steer_bias = 0;
    if (got_right) {
        wall_invalid_count = 0;
        float error_right = wall_target_right_mm - right_dist;
        steer_bias = constrain(
            (int)lroundf(wall_right_kp * error_right),
            -wall_max_steer, wall_max_steer
        );
    } else if (++wall_invalid_count > wall_invalid_limit) {
        finish_wall_follow_run(4, true);  // RIGHT_INVALID — turned a corner
        return;
    }

    motorsForwardSteered(wall_base_pwm, steer_bias);
    append_wall_log(wall_last_front_mm, wall_last_right_mm, steer_bias,
                    wall_last_left_pwm, wall_last_right_pwm, now);
}
```

</div>
</details>

<details>
<summary>Python: wall-follow command dispatch and poll</summary>
<div markdown="1">

```python
def drive_wall_follow_segment(base_pwm, target_right_mm, kp, front_stop_mm,
                              front_safety_mm, timeout_ms, max_steer, invalid_limit):
    ble.send_command(
        CMD.WALL_FOLLOW_START,
        f'{base_pwm}|{target_right_mm:.1f}|{kp:.4f}|{front_stop_mm:.1f}'
        f'|{front_safety_mm:.1f}|{timeout_ms}|{max_steer}|{invalid_limit}'
    )
    deadline = time.time() + timeout_ms / 1000.0 + 2.0
    while time.time() < deadline:
        ble.send_command(CMD.GET_WALL_STATUS, '')
        msg = ble.receive_string(ble.uuid['RX_STRING']).strip()
        if msg.startswith('WALL_DONE|'):
            parts = msg.split('|')
            return {'reason': parts[1], 'log_pos': int(parts[2])}
        status = robot._parse_wall_status(msg)
        if status['state'] == 'IDLE' and status['reason'] != 'NONE':
            return status
        time.sleep(0.15)
    raise TimeoutError('wall-follow timed out')
```

</div>
</details>

## Simulation Validation

Before testing on hardware I ran the full strategy in simulation to verify the logic. The simulator executes the same waypoint sequence with the same localization code.

<div style="display:flex; gap:16px; align-items:flex-start; flex-wrap:wrap;">
  <div style="flex:1; min-width:300px; text-align:center;">
    <video width="100%" controls>
      <source src='/images/mae4190/lab12/validation_in_sim.mp4' type='video/mp4'>
    </video>
    <div style="font-size:0.9em; color:#555;">Simulation run through all 8 segments.</div>
  </div>
  <div style="flex:1; min-width:300px; text-align:center;">
    <img src='/images/mae4190/lab12/validation_in_sim_traj.png' width='100%'>
    <div style="font-size:0.9em; color:#555;">Simulated GT path vs desired waypoints. Bayes filter localization at segment 3 checkpoint converged at probability 1.0.</div>
  </div>
</div>

## Failures and Iteration

The first wall-following attempts failed because the right-sensor P gain was too low. Corrections came too late and the robot drifted into the wall before the steer bias could pull it back.

<div style="display:flex; gap:16px; align-items:flex-start; flex-wrap:wrap;">
  <div style="flex:1; min-width:280px; text-align:center;">
    <video width="100%" controls>
      <source src='/images/mae4190/lab12/follow_wall_not_good.mp4' type='video/mp4'>
    </video>
    <div style="font-size:0.9em; color:#555;">Attempt 1: robot clips the wall.</div>
  </div>
  <div style="flex:1; min-width:280px; text-align:center;">
    <video width="100%" controls>
      <source src='/images/mae4190/lab12/follow_wall_not_good_2.mp4' type='video/mp4'>
    </video>
    <div style="font-size:0.9em; color:#555;">Attempt 2: wrong wall follow contorl lead robot early into final waypoint, then hit the wall.</div>
  </div>
</div>

## Making the Right Sensor Robust

The core problem was that a small `wall_right_kp` produced a small `steer_bias`, so the robot could be 100mm off from its target standoff and still receive only a gentle correction. By the time the correction accumulated to a meaningful steering change, the robot had already closed the gap. Increasing `wall_right_kp` made the correction proportional to the actual position error so that a larger offset produced a strong immediate steer. The `wall_max_steer` cap prevents the gain from causing overcorrection oscillation once the robot is close to the target standoff.

<details>
<summary>Arduino: right-sensor proportional steer calculation</summary>
<div markdown="1">

```cpp
float error_right = wall_target_right_mm - right_dist;

// steer_bias > 0  →  more left motor, robot steers right (toward wall)
// steer_bias < 0  →  more right motor, robot steers left (away from wall)
steer_bias = constrain(
    (int)lroundf(wall_right_kp * error_right),
    -wall_max_steer, wall_max_steer
);

motorsForwardSteered(wall_base_pwm, steer_bias);
```

</div>
</details>

<details>
<summary>Python: wall-follow gain parameters sent to Artemis</summary>
<div markdown="1">

```python
WALL_KP       = 0.14   # right-sensor P gain — tuned until corrections were fast enough
WALL_MAX_STEER = 25    # max PWM delta; caps the gain to prevent oscillation

# each segment's front_stop_mm and front_safety_mm come from
# the theoretical ray-cast distance at the waypoint endpoint,
# so no hard distances are ever written by hand
seg['front_stop_mm'] = max(220, int(round(front_stop_scale * geom['end_front_mm'])))

status = drive_wall_follow_segment(
    seg['base_pwm'], seg['target_right_mm'],
    WALL_KP, seg['front_stop_mm'],
    seg.get('front_safety_mm', WALL_FRONT_SAFETY_MM),
    seg['timeout_ms'], WALL_MAX_STEER, WALL_INVALID_LIMIT
)
```

</div>
</details>

After the tuning, segments 6 and 8 stopped cleanly on a front-distance event. Segments 4 and 7 stopped on a right-invalid event, which happens correctly when the robot rounds a corner and the right sensor briefly loses the wall.

## Final Run

The final run executes all 8 segments in sequence. The open-loop phase brings the robot to approximately (2,-3). The wall-follower picks up the bottom wall and carries it to (5,-3), steps north to (5,-2), follows the east wall up to (5,3), follows the top wall west to (0,3), and then follows the center wall south to (0,0).

Waypoints 1 through 3 are hit approximately since odometry accumulates some heading and distance error over those diagonal and lateral segments, but the error is small enough over those distances that the robot arrives within roughly one grid cell of each target. Waypoints 4 through 9 are much more accurate because the ToF sensors correct position continuously throughout each segment. The final position at (0,0) landed within about half a foot of the target.

<video width="700" controls>
  <source src='/images/mae4190/lab12/final.mp4' type='video/mp4'>
</video>

Meet my cat Mulberry! 🐱

<div>
  <img src='/images/mae4190/cats/cat1.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat2.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat3.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat4.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat5.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat6.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat7.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat8.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat9.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat10.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat11.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat12.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat13.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat14.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat15.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat16.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat17.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat18.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat19.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
  <img src='/images/mae4190/cats/cat20.png' style="max-width:300px; height:auto; margin:4px; display:inline-block; vertical-align:top;">
</div>

[Back to MAE 4190](/mae4190/)
