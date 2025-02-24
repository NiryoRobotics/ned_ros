#!/usr/bin/env python3

from niryo_robot_python_ros_wrapper import NiryoRosWrapper, JointsPosition

# Instantiate the ROS wrapper and initialize the ROS node
robot = NiryoRosWrapper.init_with_node()

# Calibrate if needed
robot.calibrate_auto()  # Only for Ned2

# Move the robot
robot.move(JointsPosition(0.1, -0.2, 0.0, 1.1, -0.5, 0.2))
