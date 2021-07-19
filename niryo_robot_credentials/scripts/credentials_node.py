#!/usr/bin/env python

# Libs
import rospy
import os

from niryo_robot_credentials.SerialNumber import SerialNumber
from niryo_robot_credentials.ApiKey import ApiKey

# Message / Service
from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_credentials.srv import GetCredential, SetCredential


class CredentialsNode:
    def __init__(self):
        rospy.logdebug("Credentials Node - Entering in Init")

        serial_number_file_path = os.path.expanduser(rospy.get_param("~serial_number_file_path"))
        api_key_file_path = os.path.expanduser(rospy.get_param('~api_key_file_path'))
        self.__serial_number = SerialNumber(serial_path=serial_number_file_path)
        self.__api_key = ApiKey(api_key_file_path=api_key_file_path)

        # Service
        rospy.Service('~get_serial', GetCredential, self.__callback_get_serial)
        rospy.Service('~get_api_key', GetCredential, self.__callback_get_api_key)
        rospy.Service('~set_api_key', SetCredential, self.__callback_set_api_key)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("Credentials Node - Node Started")

    def __callback_get_serial(self, _req):
        try:
            serial_number = self.__serial_number.read_serial()
        except OSError:
            rospy.logerr("Credentials Node - Unable to read serial")
            return CommandStatus.CREDENTIALS_FILE_ERROR, "File error"
        rospy.logdebug("Credentials Node - Serial Read")

        if not serial_number:
            rospy.logerr("Credentials Node - Serial file is empty")
            return CommandStatus.CREDENTIALS_UNKNOWN_ERROR, "No serial"
        return CommandStatus.SUCCESS, serial_number

    def __callback_get_api_key(self, _req):
        api_key = self.__api_key.read_key()
        if api_key is None:
            return CommandStatus.CREDENTIALS_FILE_ERROR, "Couldn't access the file"
        return CommandStatus.SUCCESS, api_key

    def __callback_set_api_key(self, req):
        success = self.__api_key.write_key(req.credential)
        if not success:
            return CommandStatus.CREDENTIALS_FILE_ERROR, "Couldn't access the file"
        return CommandStatus.SUCCESS, 'The API key has been successfully set'

if __name__ == "__main__":
    rospy.init_node('niryo_robot_credentials', anonymous=False, log_level=rospy.INFO)
    try:
        node = CredentialsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
