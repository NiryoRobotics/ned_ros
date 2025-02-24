#!/usr/bin/env python3

from niryo_robot_python_ros_wrapper import NiryoRosWrapper, Pose

import math

# Instantiate the ROS wrapper and initialize the ROS node
robot = NiryoRosWrapper.init_with_node()

# Calibrate if needed
robot.calibrate_auto()  # Only for Ned2

# Setup the tool
robot.update_tool()

# Open the tool
robot.release_with_tool()

# Move to pick pose
robot.move(Pose(0.2, 0.1, 0.14, 0.0, math.pi, -math.pi / 2))

# Pick
robot.grasp_with_tool()

# Move to place pose
robot.move(Pose(0.2, -0.1, 0.14, 0.0, math.pi, -math.pi / 2))

# Place
robot.release_with_tool()
