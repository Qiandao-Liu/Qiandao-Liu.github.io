---
layout: archive
title: "Lab 12: Planning and Execution"
permalink: /mae4190/lab12/
author_profile: true
---

{% include base_path %}

## ECE Robotics Day

The robot also performed at ECE Robotics Day before lab 12 was due, doing the same drift stunt from Lab 8. Same firmware, same robot. It was a good reminder that the platform is solid.

<video width="700" controls>
  <source src='/images/mae4190/lab12/ECE_Robotics_Day_drift_show.mp4' type='video/mp4'>
</video>

## Strategy

The task is to navigate through 9 fixed waypoints on a known map. I did not implement global path planning. All the waypoints are given in advance and nothing in the environment moves, so there is nothing to plan online. Running the laptop as a real-time planner would also be slow since BLE communication adds too much latency for closed-loop corrections to be useful.

The path splits naturally into two phases. Segments 1 through 3 travel through open space at diagonal and lateral angles where wall references are not reliable, so I used gyroscope-based orientation control with timed forward drive. Segments 4 through 8 run along or close to the surrounding walls, so I switched to a wall-following controller using both the front and right ToF sensors. All motion decisions execute onboard on the Artemis. The laptop only triggers segments and runs the Bayes filter localization computation.

## Open-Loop Phase

The first three segments move from (-4,-3) to (-2,-1), then to (1,-1), then down to (2,-3). For each segment the robot first aligns to the correct heading using a gyroscope PID loop, then drives forward for a duration proportional to the segment length. The front ToF sensor watches for unexpected obstacles throughout and stops the drive early if the robot gets too close to a wall.

IMU drift over these short distances is small enough that the position error at waypoint 3 is acceptable as a handoff into wall-following. The heading control is reliable since the gyroscope is read directly. The geometry of waypoints 1 through 3 and the surrounding walls makes it difficult to use a consistent right-sensor reference anyway, so timed drive is the pragmatic choice here.

## Initial Localization

I ran one Bayes filter update at the very start to verify the robot's actual placement at (-4,-3). Hand-placement is never exact. The filter used a weak spatial prior centered on the expected starting pose, then updated from a full 360-degree scan.

The update converged at a belief probability of 0.9986 to the grid cell at (-4.0 ft, -3.0 ft) with an estimated heading of -10 degrees. The position match was essentially exact; the small heading offset was noted and factored into the first turn command.

<img src='/images/mae4190/lab12/first_localization_pos.png' width='700'>

I originally planned a second localization checkpoint after segment 3, before entering the wall-following phase, as an extra correction step.

<img src='/images/mae4190/lab12/second_localization_pos.png' width='700'>

In the final run I removed it. The wall-following controller corrects lateral offset within the first meter of each wall segment, so any drift from the open-loop phase gets absorbed quickly. The scan time was not worth the recovery benefit.

## Wall-Following Phase

Professor Petersen mentioned in lecture that a robot can use its right-side sensor to track a wall. Segments 4, 6, 7, and 8 all travel along walls, so I built the controller around that idea.

The right ToF sensor feeds a P controller that steers left or right to hold a target standoff from the right wall. The front ToF sensor feeds a separate PID controller that modulates forward speed and triggers a stop when the distance ahead drops to the expected endpoint distance. This keeps the robot at a consistent lateral offset during straight runs and stops it accurately without any fixed timing from the laptop side. The steering and speed controllers run together in the onboard loop so the robot can handle small wall irregularities and still arrive at the right place.

Segment 5 between (5,-3) and (5,-2) is a short one-foot repositioning step. It moves the robot into position so the right sensor is against the wall that segment 6 will follow. It runs open-loop since the gap is too short for useful wall tracking.

## Simulation Validation

Before testing on hardware I ran the full strategy in simulation to verify the logic. The simulator executes the same waypoint sequence with the same localization code.

<video width="700" controls>
  <source src='/images/mae4190/lab12/validation_in_sim.mp4' type='video/mp4'>
</video>

<img src='/images/mae4190/lab12/validation_in_sim_traj.png' width='700'>

The simulated ground-truth path tracked the desired waypoints closely through all 8 segments. The Bayes filter localization at the checkpoint after segment 3 converged with probability 1.0 to a cell within 0.3 ft of the actual position. This confirmed the handoff logic was sound before I put the real robot on the floor.

## Failures and Iteration

The first wall-following attempts were not good. The right-sensor proportional gain was too low, so corrections came too late and the robot drifted into the wall.

<video width="700" controls>
  <source src='/images/mae4190/lab12/follow_wall_not_good.mp4' type='video/mp4'>
</video>

<video width="700" controls>
  <source src='/images/mae4190/lab12/follow_wall_not_good_2.mp4' type='video/mp4'>
</video>

After tuning the gains the robot held a consistent standoff through all wall segments. Segments 6 and 8 stopped on a front-distance threshold as expected. Segments 4 and 7 stopped on right-invalid events, which happen when the robot turns a corner and the right sensor briefly loses the wall. Both stop conditions are handled onboard without any laptop intervention.

## Final Run

The final run executes all 8 segments in sequence. The open-loop phase brings the robot to approximately (2,-3). The wall-follower then picks up the bottom wall and carries it to (5,-3), steps north to (5,-2), follows the east wall up to (5,3), follows the top wall west to (0,3), and then follows the center wall south to (0,0).

Waypoints 1 through 3 are hit approximately. Odometry accumulates some error over the diagonal and horizontal segments, but the error is small enough over those distances that the robot arrives within roughly one grid cell of each target. Waypoints 4 through 9 are much more accurate because the ToF sensors continuously correct position throughout each segment. The final position at (0,0) landed within about half a foot of the target.

<video width="700" controls>
  <source src='/images/mae4190/lab12/final.mp4' type='video/mp4'>
</video>

Meet my cat Mulberry! 🐱

<div style="display:flex; flex-wrap:wrap; gap:8px;">
  <img src='/images/mae4190/cats/cat1.png' width='280'>
  <img src='/images/mae4190/cats/cat2.png' width='280'>
  <img src='/images/mae4190/cats/cat3.png' width='280'>
  <img src='/images/mae4190/cats/cat4.png' width='280'>
  <img src='/images/mae4190/cats/cat5.png' width='280'>
  <img src='/images/mae4190/cats/cat6.png' width='280'>
  <img src='/images/mae4190/cats/cat7.png' width='280'>
  <img src='/images/mae4190/cats/cat8.png' width='280'>
  <img src='/images/mae4190/cats/cat9.png' width='280'>
  <img src='/images/mae4190/cats/cat10.png' width='280'>
  <img src='/images/mae4190/cats/cat11.png' width='280'>
  <img src='/images/mae4190/cats/cat12.png' width='280'>
  <img src='/images/mae4190/cats/cat13.png' width='280'>
  <img src='/images/mae4190/cats/cat14.png' width='280'>
  <img src='/images/mae4190/cats/cat15.png' width='280'>
  <img src='/images/mae4190/cats/cat16.png' width='280'>
  <img src='/images/mae4190/cats/cat17.png' width='280'>
  <img src='/images/mae4190/cats/cat18.png' width='280'>
  <img src='/images/mae4190/cats/cat19.png' width='280'>
  <img src='/images/mae4190/cats/cat20.png' width='280'>
</div>

[Back to MAE 4190](/mae4190/)
