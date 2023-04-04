#!/bin/bash

if [ "$#" -ne 7 ]; then
	echo "Error: There should be 7 parameters."
	exit -1;
fi;
ros2 topic pub /command \
trajectory_msgs/msg/JointTrajectoryPoint \
"{positions: [$1, $2, $3, $4, $5, $6, $7], \
velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
accelerations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  \
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
time_from_start: {sec: 0, nanosec: 0}}" "-1"
