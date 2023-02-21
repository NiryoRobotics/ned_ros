#!/usr/bin/env python

import rospy
import logging

from niryo_robot_msgs.msg import CommandStatus
from niryo_robot_system_api_client.msg import WifiStatus

from niryo_robot_msgs.srv import SetString

from niryo_robot_system_api_client.HttpClient import HttpClient


class SystemApiClientNode:

    def __init__(self):
        rospy.logdebug("System Api Client - Entering in Init")
        self.client = HttpClient()

        self.set_robot_name_server = rospy.Service('/niryo_robot/wifi/set_robot_name',
                                                   SetString,
                                                   self.__callback_set_robot_name)

        self.wifi_state_publisher = rospy.Publisher('/niryo_robot/wifi/status', WifiStatus, queue_size=2)
        self.slow_mode = False
        self.slow_mode_timer = rospy.Duration(rospy.get_param('~slow_mode_timer'))
        self.normal_mode_timer = rospy.Duration(rospy.get_param('~normal_mode_timer'))

        self.hotspot_timer = rospy.Timer(self.normal_mode_timer, self.__publish_hotspot_state_callback)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("System Api Client - Node Started")

    def __publish_hotspot_state_callback(self, _):
        conn_success, status = self.client.wifi_state()

        # In case of connection error we increase the timer interval to avoid useless requests
        if not conn_success:
            self.slow_mode = True
            self.hotspot_timer.shutdown()
            self.hotspot_timer = rospy.Timer(self.slow_mode_timer, self.__publish_hotspot_state_callback)
        elif self.slow_mode:
            self.slow_mode = False
            self.hotspot_timer.shutdown()
            self.hotspot_timer = rospy.Timer(self.normal_mode_timer, self.__publish_hotspot_state_callback)

        self.__publish_hotspot_state(conn_success, status)

    def __publish_hotspot_state(self, conn_success, status):
        msg = WifiStatus()
        if not conn_success:
            msg.hotspot_status = msg.UNKNOWN
            msg.wlan0_status = msg.UNKNOWN
        else:
            msg.hotspot_status = msg.ON if status['hotspot_state'] else msg.OFF
            msg.wlan0_status = msg.ON if status['wlan0_state'] else msg.OFF
        try:
            self.wifi_state_publisher.publish(msg)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node('niryo_robot_system_api_client', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    SystemApiClientNode()
    rospy.spin()
