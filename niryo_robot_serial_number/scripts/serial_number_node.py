#!/usr/bin/env python

# Libs
import rospy
import os

from niryo_robot_serial_number.SerialNumber import SerialNumber

# Message / Service
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_serial_number.srv import GetSerial


class SerialNumberNode:
    def __init__(self):
        rospy.logdebug("Serial Number Node - Entering in Init")

        serial_path = os.path.expanduser(rospy.get_param("~serial_path"))
        self.__serial_number = SerialNumber(serial_path=serial_path)

        # Service
        rospy.Service('~get', GetSerial,
                      self.__callback_get_serial)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("Serial Number Node - Node Started")

    def __callback_get_serial(self, _req):
        try:
            serial_number = self.__serial_number.read_serial()
        except OSError:
            rospy.logerr("Serial Number Node - Unable to read serial")
            return CommandStatus.SERIAL_FILE_ERROR, "File error"
        rospy.logdebug("Serial Number Node - Serial Read")

        if not serial_number:
            rospy.logerr("Serial Number Node - Serial file is empty")
            return CommandStatus.SERIAL_UNKNOWN_ERROR, "No serial"
        return CommandStatus.SUCCESS, serial_number


if __name__ == "__main__":
    rospy.init_node('niryo_robot_serial_number', anonymous=False, log_level=rospy.INFO)
    try:
        node = SerialNumberNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
