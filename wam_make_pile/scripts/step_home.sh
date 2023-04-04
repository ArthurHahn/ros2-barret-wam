#!/bin/bash

ros2 topic pub /command \
trajectory_msgs/msg/JointTrajectoryPoint \
"{positions: [0.0, -2.0, 0.0, 3.1, 0.0, 0.0, 0.0], \
velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  \
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
time_from_start: {sec: 0, nanosec: 0}}" "-1"
