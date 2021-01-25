#!/usr/bin/env python

import rospy
from niryo_robot_system_api_client.HttpClient import HttpClient

from std_msgs.msg import Bool
from niryo_robot_msgs.srv import SetInt
from niryo_robot_msgs.srv import SetString
from niryo_robot_msgs.msg import CommandStatus


class SystemApiClientNode:
    def __init__(self):
        rospy.logdebug("System Api Client - Entering in Init")

        domain = rospy.get_param('~server_domain')
        port = rospy.get_param('~server_port')

        self.client = HttpClient(domain, port)

        self.set_robot_name_server = rospy.Service('/niryo_robot/wifi/set_robot_name', SetString,
                                                   self.__callback_set_robot_name)

        self.hotspot_state_publisher = rospy.Publisher('/niryo_robot/wifi/hotspot', Bool, queue_size=2)
        self.slow_mode = False
        self.hotspot_timer = rospy.Timer(rospy.Duration(1), self.__publish_hotspot_state)

        self.activate_hotspot_server = rospy.Service('/niryo_robot/wifi/set_hotspot', SetInt,
                                                     self.__callback_activate_hotspot)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("System Api Client - Node Started")

    def __callback_set_robot_name(self, req):
        name = req.value
        rospy.logdebug("System Api Client - Setting robot name to: " + str(name))
        if len(name) > 32 or len(name) < 3:
            rospy.logwarn('System Api Client - Invalid name: length must be between 3-32 characters')
            return CommandStatus.SYSTEM_API_CLIENT_INVALID_ROBOT_NAME, 'Name length must be between 3-32 characters'

        conn_success, result = self.client.set_robot_name(name)
        if not conn_success:
            return CommandStatus.SYSTEM_API_CLIENT_REQUEST_FAILED, result
        if not result:
            return CommandStatus.SYSTEM_API_CLIENT_UNKNOWN_ERROR, 'Could not change the robot name'
        return CommandStatus.SUCCESS, 'Successfully changed the robot name'

    def __publish_hotspot_state(self, _):
        conn_success, activated = self.client.hotspot_state()

        # In case of connection error we increase the timer interval to avoid useless requests
        if not conn_success:
            self.slow_mode = True
            self.hotspot_timer.shutdown()
            self.hotspot_timer = rospy.Timer(rospy.Duration(5), self.__publish_hotspot_state)

        if conn_success and self.slow_mode:
            self.slow_mode = False
            self.hotspot_timer.shutdown()
            self.hotspot_timer = rospy.Timer(rospy.Duration(1), self.__publish_hotspot_state)

        msg = Bool()
        if conn_success:
            msg.data = activated
        else:
            msg.data = False
        try:
            self.hotspot_state_publisher.publish(msg)
        except rospy.ROSException:
            return

    def __callback_activate_hotspot(self, _):
        conn_success, result = self.client.activate_hotspot()

        if not conn_success:
            return CommandStatus.SYSTEM_API_CLIENT_REQUEST_FAILED, result
        if not result:
            return CommandStatus.SYSTEM_API_CLIENT_UNKNOWN_ERROR, "Failed to activate hotspot mode"

        return CommandStatus.SUCCESS, "Hotspot mode activated"


if __name__ == "__main__":
    rospy.init_node('niryo_robot_system_api_client', anonymous=False, log_level=rospy.INFO)
    SystemApiClientNode()
    rospy.spin()
