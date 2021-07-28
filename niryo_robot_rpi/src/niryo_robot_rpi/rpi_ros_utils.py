#!/usr/bin/env python

import rospy
import rospkg
import subprocess
import re

from niryo_robot_msgs.srv import SetBool, SetInt, Trigger
from niryo_robot_msgs.msg import CommandStatus

ENABLE_BUS_MOTORS_SUCCESS = 1
ENABLE_BUS_MOTORS_READ_FAIL = -1
ENABLE_BUS_MOTORS_WRITE_FAIL = -2

CHANGE_MOTOR_CONFIG_SUCCESS = 1
CHANGE_MOTOR_CONFIG_READ_FAIL = -1
CHANGE_MOTOR_CONFIG_WRITE_FAIL = -2
CHANGE_MOTOR_CONFIG_WRONG_VERSION = -3


class LedState:
    def __init__(self):
        pass

    SHUTDOWN = 1
    HOTSPOT = 2
    HW_ERROR = 3
    OK = 4
    WAIT_HOTSPOT = 5
    PAUSE = 6


def send_hotspot_command():
    rospy.loginfo("HOTSPOT")
    send_led_state(LedState.WAIT_HOTSPOT)
    rospy.wait_for_service('/niryo_robot/wifi/set_hotspot', timeout=0.5)
    try:
        set_hotspot = rospy.ServiceProxy('/niryo_robot/wifi/set_hotspot', SetInt)
        set_hotspot()
    except rospy.ServiceException:
        rospy.logwarn("Could not call set_hotspot service")


def send_trigger_program_autorun():
    rospy.loginfo("Trigger program autorun from button")
    topic_name = "/niryo_robot_programs_manager/execute_program_autorun"
    try:
        rospy.wait_for_service(topic_name, 0.1)
        trigger = rospy.ServiceProxy(topic_name, Trigger)
        resp = trigger()
        return resp.status, resp.message
    except (rospy.ServiceException, rospy.ROSException):
        return CommandStatus.FAILURE, "Send trigger autorun error"


def send_reboot_motors_command():
    rospy.loginfo("Send reboot motor command")
    try:
        rospy.wait_for_service('/niryo_robot/reboot_motors', timeout=0.5)
    except rospy.ROSException:
        pass
    try:
        reboot_motors = rospy.ServiceProxy('/niryo_robot/reboot_motors', Trigger)
        reboot_motors()
    except rospy.ServiceException:
        pass


def send_shutdown_command():
    rospy.loginfo("SHUTDOWN")
    send_led_state(LedState.SHUTDOWN)
    rospy.loginfo("Activate learning mode")
    try:
        rospy.wait_for_service('/niryo_robot/learning_mode/activate', timeout=1)
    except rospy.ROSException:
        pass
    try:
        rospy.ServiceProxy('/niryo_robot/learning_mode/activate', SetBool)(True)
    except rospy.ServiceException:
        pass
    send_reboot_motors_command()
    rospy.sleep(0.2)
    rospy.loginfo("Command 'sudo shutdown now'")
    try:
        output = subprocess.check_output(['sudo', 'shutdown', 'now'])
        rospy.loginfo(str(output))
    except subprocess.CalledProcessError:
        rospy.logwarn("Can't exec shutdown cmd")


def send_reboot_command():
    rospy.loginfo("REBOOT")
    send_led_state(LedState.SHUTDOWN)
    rospy.loginfo("Activate learning mode")
    try:
        rospy.wait_for_service('/niryo_robot/learning_mode/activate', timeout=0.5)
    except rospy.ROSException:
        pass
    try:
        rospy.ServiceProxy('/niryo_robot/learning_mode/activate', SetBool)(True)
    except rospy.ServiceException:
        pass
    send_reboot_motors_command()
    rospy.sleep(0.2)
    rospy.loginfo("Command 'sudo reboot'")
    try:
        output = subprocess.check_output(['sudo', 'reboot'])
        rospy.loginfo(str(output))
    except subprocess.CalledProcessError:
        rospy.logwarn("Can't exec reboot cmd")


def send_led_state(state):
    rospy.wait_for_service('/niryo_robot/rpi/set_led_state', timeout=0.5)
    try:
        set_led = rospy.ServiceProxy('/niryo_robot/rpi/set_led_state', SetInt)
        set_led(state)
    except rospy.ServiceException:
        rospy.logwarn("Could not call set_led_state service")


def activate_learning_mode(activate):
    try:
        rospy.wait_for_service('/niryo_robot/learning_mode/activate', timeout=0.5)
        rospy.ServiceProxy('/niryo_robot/learning_mode/activate', SetBool)(activate)

    except (rospy.ServiceException, rospy.ROSException) as e:
        return False
    return True


def stop_robot_action():
    # Stop current move command
    try:
        rospy.wait_for_service('/niryo_robot_arm_commander/stop_command', timeout=0.5)
        stop_cmd = rospy.ServiceProxy('/niryo_robot_arm_commander/stop_command', Trigger)
        stop_cmd()
    except (rospy.ServiceException, rospy.ROSException) as e:
        pass
