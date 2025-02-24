#!/usr/bin/env python3

from niryo_robot_python_ros_wrapper import NiryoRosWrapper

# Instantiate the ROS wrapper and initialize the ROS node
robot = NiryoRosWrapper.init_with_node()

# Calibrate if needed
robot.calibrate_auto()  # Only for Ned2

# Setup the tool
robot.update_tool()

# Release
robot.release_with_tool()
