#!/usr/bin/env python

import rospy

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Services
from niryo_robot_rpi.srv import SetDigitalIO

from tools_interface.srv import PingDxlTool
from tools_interface.srv import OpenGripper, CloseGripper
from tools_interface.srv import PullAirVacuumPump, PushAirVacuumPump


class ToolRosCommandInterface:

    def __init__(self, state_ros_communication_problem):
        namespace = rospy.get_param("~namespace_topics")
        self.__service_open_gripper = rospy.ServiceProxy(namespace + 'open_gripper',
                                                         OpenGripper)
        self.__service_close_gripper = rospy.ServiceProxy(namespace + 'close_gripper',
                                                          CloseGripper)

        self.__service_pull_air_vacuum_pump = rospy.ServiceProxy(namespace + 'pull_air_vacuum_pump',
                                                                 PullAirVacuumPump)
        self.__service_push_air_vacuum_pump = rospy.ServiceProxy(namespace + 'push_air_vacuum_pump',
                                                                 PushAirVacuumPump)

        self.__service_ping_dxl_tool = rospy.ServiceProxy(namespace + 'ping_and_set_dxl_tool', PingDxlTool)

        self.__service_setup_digital_output_tool = rospy.ServiceProxy('/niryo_robot_rpi/set_digital_io_mode',
                                                                      SetDigitalIO)
        self.__service_activate_digital_output_tool = rospy.ServiceProxy('/niryo_robot_rpi/set_digital_io_state',
                                                                         SetDigitalIO)

        self.__state_ros_communication_problem = state_ros_communication_problem
        rospy.logdebug("Interface between Tools Commander and ROS Control has been started.")

    # Gripper

    def open_gripper(self, gripper_id, open_position, open_speed, open_hold_torque):
        try:
            resp = self.__service_open_gripper(gripper_id, open_position, open_speed, open_hold_torque)
            return resp.state
        except rospy.ServiceException:
            rospy.logerr("ROS Tool Interface - Failed to Open Gripper")
            return self.__state_ros_communication_problem

    def close_gripper(self, gripper_id, close_position, close_speed, close_hold_torque, close_max_torque):
        try:
            resp = self.__service_close_gripper(gripper_id, close_position, close_speed, close_hold_torque,
                                                close_max_torque)
            return resp.state
        except rospy.ServiceException:
            rospy.logerr("ROS Tool Interface - Failed to Close Gripper")
            return self.__state_ros_communication_problem

    # Vacuum
    def pull_air_vacuum_pump(self, vp_id, vp_pull_air_position, vp_pull_air_hold_torque):
        try:
            resp = self.__service_pull_air_vacuum_pump(vp_id, vp_pull_air_position, vp_pull_air_hold_torque)
            return resp.state
        except rospy.ServiceException:
            rospy.logerr("ROS Tool Interface - Failed to Pull Air")
            return self.__state_ros_communication_problem

    def push_air_vacuum_pump(self, vp_id, vp_push_air_position):
        try:
            resp = self.__service_push_air_vacuum_pump(vp_id, vp_push_air_position)
            return resp.state
        except rospy.ServiceException:
            rospy.logerr("ROS Tool Interface - Failed to Push Air")
            return self.__state_ros_communication_problem

    # Others
    def ping_dxl_tool(self):
        try:
            resp = self.__service_ping_dxl_tool()
            return resp.state, resp.id
        except rospy.ServiceException:
            rospy.logerr("ROS Tool Interface - Failed to Ping Dynamixel Tool - An error has occurred or another ping was running")
            return CommandStatus.TOOL_ROS_INTERFACE_ERROR, "Cannot ping dxl tool"

    def digital_output_tool_setup(self, gpio_pin):
        try:
            resp = self.__service_setup_digital_output_tool(gpio_pin, 0)  # set output
            return resp.status, resp.message
        except rospy.ServiceException:
            rospy.logerr("ROS Tool Interface - Failed to get digital output setup")
            return CommandStatus.TOOL_ROS_INTERFACE_ERROR, "Digital IO panel service failed"

    def digital_output_tool_activate(self, gpio_pin, activate):
        try:
            resp = self.__service_activate_digital_output_tool(gpio_pin, activate)
            return resp.status, resp.message
        except rospy.ServiceException:
            rospy.logerr("ROS Tool Interface - Failed to activate digital output")
            return CommandStatus.TOOL_ROS_INTERFACE_ERROR, "Digital IO panel service failed"
