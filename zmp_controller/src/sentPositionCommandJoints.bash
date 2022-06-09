#!/bin/bash

# rostopic pub -r 10 /right_leg_controller/command trajectory_msgs/JointTrajectory "header: 
#   seq: 0
#   stamp: 
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# joint_names: ['leg_right_1_joint', 'leg_right_2_joint', 'leg_right_3_joint', 'leg_right_4_joint', 'leg_right_5_joint', 'leg_right_6_joint']
# points: 
#   - 
#     positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#     velocities: []
#     accelerations: []
#     effort: []
#     time_from_start: 
#       secs: 0
#       nsecs: 10"


rostopic pub -r 10 /left_leg_controller/command trajectory_msgs/JointTrajectory "header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['leg_left_1_joint', 'leg_left_2_joint', 'leg_left_3_joint', 'leg_left_4_joint', 'leg_left_5_joint', 'leg_left_6_joint']
points: 
  - 
    positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    accelerations: []
    effort: []
    time_from_start: 
      secs: 0
      nsecs: 10"

rostopic pub -r 10 /right_leg_controller/command trajectory_msgs/JointTrajectory "header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['leg_right_1_joint', 'leg_right_2_joint', 'leg_right_3_joint', 'leg_right_4_joint', 'leg_right_5_joint', 'leg_right_6_joint']
points: 
  - 
    positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0]
    velocities: []
    accelerations: []
    effort: []
    time_from_start: 
      secs: 0
      nsecs: 10"

rostopic pub -r 10 /left_arm_controller/command trajectory_msgs/JointTrajectory "header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']
points: 
  - 
    positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocities: []
    accelerations: []
    effort: []
    time_from_start: 
      secs: 2
      nsecs: 0"

rostopic pub -r 10 /left_arm_controller/command trajectory_msgs/JointTrajectory "header: 
  seq: 0
  stamp: 
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']
points: 
  - 
    positions: [0.83, 0.11, -0.4, 1.22, 2.04, 0.36, -0.20]
    velocities: []
    accelerations: []
    effort: []
    time_from_start: 
      secs: 2
      nsecs: 0"