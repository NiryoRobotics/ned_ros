#!/usr/bin/env python3

from niryo_robot_python_ros_wrapper import NiryoRosWrapper, Pose, ObjectShape, ObjectColor

import math

# - Constants

# Robot's Workspace Name, replace by your own workspace name
WORKSPACE_NAME = "workspace_1"
# The observation pose
OBSERVATION_POSE = Pose(0.18, 0.0, 0.35, math.pi, 0.0, 0.0)
# The Place pose
PLACE_POSE = Pose(0.0, -0.25, 0.1, math.pi, 0.0, -math.pi / 2)

# Instantiate the ROS wrapper and initialize the ROS node
robot = NiryoRosWrapper.init_with_node()

# Calibrate if needed
robot.calibrate_auto()  # Only for Ned2

# Setup the tool
robot.update_tool()

# Move to observation pose
robot.move(OBSERVATION_POSE)

# Perform a vision pick
result = robot.vision_pick(WORKSPACE_NAME, height_offset=0.0, shape=ObjectShape.ANY, color=ObjectColor.ANY)
object_found, shape, color = result

if object_found:
    print("Object found, with shape: {}, and color: {}".format(shape, color))
    robot.place(PLACE_POSE)
else:
    print("No object found")
