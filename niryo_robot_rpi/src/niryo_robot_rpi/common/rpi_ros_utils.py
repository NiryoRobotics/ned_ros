# rpi_ros_utils.py
# Copyright (C) 2021 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
from pathlib import Path
import subprocess

from niryo_robot_sound.srv import PlaySound, PlaySoundRequest

from niryo_robot_msgs.srv import SetBool, Trigger
from niryo_robot_msgs.srv import SetInt
from niryo_robot_msgs.msg import CommandStatus

__all__ = [
    "LedState",
    "send_trigger_program_autorun",
    "send_reboot_motors_command",
    "send_shutdown_command",
    "send_reboot_command",
    "send_led_state",
    "activate_learning_mode",
    "auto_calibration",
    "stop_robot_action",
    "ping_i2c",
    "play_connected",
]

ENABLE_BUS_MOTORS_SUCCESS = 1
ENABLE_BUS_MOTORS_READ_FAIL = -1
ENABLE_BUS_MOTORS_WRITE_FAIL = -2

CHANGE_MOTOR_CONFIG_SUCCESS = 1
CHANGE_MOTOR_CONFIG_READ_FAIL = -1
CHANGE_MOTOR_CONFIG_WRITE_FAIL = -2
CHANGE_MOTOR_CONFIG_WRONG_VERSION = -3


class LedState:

    SHUTDOWN = 1
    HOTSPOT = 2
    HW_ERROR = 3
    OK = 4
    WAIT_HOTSPOT = 5
    PAUSE = 6


def send_trigger_program_autorun():
    rospy.loginfo("Trigger program autorun from button")
    topic_name = "/niryo_robot_programs_manager_v2/execute_program_autorun"
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


def send_shutdown_command(learning_mode=True):
    rospy.loginfo("SHUTDOWN")
    activate_learning_mode(learning_mode)
    # send_reboot_motors_command()
    rospy.sleep(0.2)
    rospy.loginfo("Command 'sudo shutdown now'")
    try:
        output = subprocess.check_output(['sudo', 'shutdown', 'now'])
        rospy.loginfo(str(output))
    except subprocess.CalledProcessError:
        rospy.logwarn("Can't exec shutdown cmd")


def send_reboot_command(learning_mode=True):
    rospy.loginfo("REBOOT")
    activate_learning_mode(learning_mode)
    # send_reboot_motors_command()
    rospy.sleep(0.2)
    rospy.loginfo("Command 'sudo reboot'")
    try:
        output = subprocess.check_output(['sudo', 'reboot'])
        rospy.loginfo(str(output))
    except subprocess.CalledProcessError:
        rospy.logwarn("Can't exec reboot cmd")


def send_led_state(state):
    if rospy.get_param("/niryo_robot_rpi/hardware_version") in ['ned2', 'ned3pro']:
        return

    try:
        rospy.wait_for_service('/niryo_robot/rpi/set_led_state', timeout=0.1)
    except rospy.ROSException:
        return

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


def auto_calibration():
    try:
        rospy.wait_for_service('/niryo_robot/joints_interface/calibrate_motors', timeout=0.5)
        rospy.ServiceProxy('/niryo_robot/joints_interface/calibrate_motors', SetInt)(1)

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


def ping_i2c(bus_num, address):
    from smbus2 import SMBus

    try:
        bus = SMBus(bus_num)
    except OSError:
        return False

    try:
        bus.read_byte(address)
        return True
    except IOError as e:
        return e.args[0] == 16  # Busy but connected if error=16


def play_connected():
    overlay_sound = rospy.ServiceProxy('/niryo_robot_sound/overlay', PlaySound)
    overlay_sound(PlaySoundRequest(sound_name='connected.wav'))
