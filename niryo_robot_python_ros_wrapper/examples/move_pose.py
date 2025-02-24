#!/usr/bin/env python3

from niryo_robot_python_ros_wrapper import NiryoRosWrapper, Pose

import math

# Instantiate the ROS wrapper and initialize the ROS node
robot = NiryoRosWrapper.init_with_node()

# Calibrate if needed
robot.calibrate_auto()  # Only for Ned2

# Move the robot
robot.move(Pose(0.25, 0.0, 0.25, 0.0, math.pi, math.pi / 2))
