#!/usr/bin/env python3

from niryo_robot_python_ros_wrapper import NiryoRosWrapper, Pose, ConveyorDirection, PinID, PinState

import math

# -- Setting variables
SENSOR_PIN_ID = PinID.DI5  # Use the PinID used by your IR sensor (DI5 is the default one of the back panel)
pick_pose = Pose(0.25, 0.14, 0.15, math.pi, 0.0, 0.0)  # Change to use meaningful values for your pick and place poses
place_pose = Pose(0.25, -0.14, 0.15, math.pi, 0.0, 0.0)

if __name__ == "__main__":

    # Instantiate the ROS wrapper and initialize the ROS node
    robot = NiryoRosWrapper.init_with_node()

    # Setup the conveyor and get its ID on the TTL bus
    conveyor_id = robot.set_conveyor()

    robot.control_conveyor(conveyor_id, bool_control_on=True, speed=50, direction=ConveyorDirection.FORWARD)

    # Wait until an object is detected by the sensor
    while robot.digital_read(SENSOR_PIN_ID) == PinState.HIGH:
        robot.wait(0.1)

    # Stop the conveyor
    robot.control_conveyor(conveyor_id, True, 0, ConveyorDirection.FORWARD)

    # Pick & place
    robot.pick_and_place(pick_pose, place_pose)

    # Deactivate the conveyor
    robot.unset_conveyor(conveyor_id)
