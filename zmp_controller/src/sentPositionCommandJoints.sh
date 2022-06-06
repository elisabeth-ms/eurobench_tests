#!/bin/bash

rostopic pub /right_arm_controller/command trajectory_msgs/JointTrajectory "header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']
points: 
  - 
    positions: [-0.44, 0.61519257357093, -2.33619449019, 0.13493568165539999, 0.0, 0.0, -0.0]
    velocities: []
    accelerations: []
    effort: []
    time_from_start: 
      secs: 2
      nsecs: 990099009" --once