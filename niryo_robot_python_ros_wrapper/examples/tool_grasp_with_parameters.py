#!/usr/bin/env python3

from niryo_robot_python_ros_wrapper import NiryoRosWrapper, ToolID, PinID

# Instantiate the ROS wrapper and initialize the ROS node
robot = NiryoRosWrapper.init_with_node()

# Calibrate if needed
robot.calibrate_auto()  # Only for Ned2

# Setup the tool
robot.update_tool()

# Get the current tool ID
current_tool_id = robot.get_current_tool_id()

if current_tool_id in [ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3]:
    robot.close_gripper(speed=500)
elif current_tool_id == ToolID.ELECTROMAGNET_1:
    electromagnet_pin = PinID.GPIO_1A  # Replace with the pin ID of your electromagnet
    robot.setup_electromagnet(electromagnet_pin)
    robot.activate_electromagnet(electromagnet_pin)
elif current_tool_id in [ToolID.VACUUM_PUMP_1, ToolID.VACUUM_PUMP_2]:
    robot.pull_air_vacuum_pump()
