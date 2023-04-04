#!/bin/bash

ros2 service call /gazebo/set_model_configuration wam joint \
"['wam_joint_1','wam_joint_2','wam_joint_3','wam_joint_4','wam_joint_5','wam_joint_6','wam_joint_7']" \
"[0.0, -2.0, 0.0, 3.1, 0.0, 0.0, 0.0]"

ros2 topic pub /command \
trajectory_msgs/msg/JointTrajectoryPoint \
"{positions: [0.0, -2.0, 0.0, 3.1, 0.0, 0.0, 0.0], \
velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  \
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
time_from_start: {sec: 0, nanosec: 0}}" "-1"
