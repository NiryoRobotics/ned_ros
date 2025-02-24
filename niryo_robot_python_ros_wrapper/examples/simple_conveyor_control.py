#!/usr/bin/env python3

from niryo_robot_python_ros_wrapper import NiryoRosWrapper, ConveyorDirection

# Instantiate the ROS wrapper and initialize the ROS node
robot = NiryoRosWrapper.init_with_node()

# Setup the conveyor and get its ID on the TTL bus
conveyor_id = robot.set_conveyor()

# Running conveyor at 50% of its maximum speed, in backward direction
robot.control_conveyor(conveyor_id, bool_control_on=True, speed=50, direction=ConveyorDirection.BACKWARD)

robot.wait(2.0)

# Running conveyor at 50% of its maximum speed, in forward direction
robot.control_conveyor(conveyor_id, bool_control_on=True, speed=50, direction=ConveyorDirection.FORWARD)

robot.wait(2.0)

# Stop the conveyor
robot.control_conveyor(conveyor_id, bool_control_on=True, speed=0, direction=ConveyorDirection.FORWARD)

# Deactivate the conveyor
robot.unset_conveyor(conveyor_id)
