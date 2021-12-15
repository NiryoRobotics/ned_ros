#!/usr/bin/env python

import rospy
import logging

from niryo_robot_system_api_client.HttpClient import HttpClient

from niryo_robot_msgs.msg import CommandStatus

from niryo_robot_msgs.srv import SetString
from niryo_robot_system_api_client.srv import ManageWifi, ManageEthernet
from niryo_robot_system_api_client.msg import WifiStatus


class SystemApiClientNode:
    def __init__(self):
        rospy.logdebug("System Api Client - Entering in Init")

        domain = rospy.get_param('~server_domain')
        port = rospy.get_param('~server_port')

        # TODO (Justin) prefix?
        prefix = None  # rospy.get_param('~api_prefix')

        rospy.logdebug("SystemApiClientNode.init - server_domain: {}".format(domain))
        rospy.logdebug("SystemApiClientNode.init - server_port: {}".format(port))

        self.client = HttpClient(domain, port, prefix=prefix)

        self.set_robot_name_server = rospy.Service('/niryo_robot/wifi/set_robot_name', SetString,
                                                   self.__callback_set_robot_name)

        self.wifi_state_publisher = rospy.Publisher('/niryo_robot/wifi/status', WifiStatus, queue_size=2)
        self.slow_mode = False
        self.slow_mode_timer = rospy.Duration(rospy.get_param('~slow_mode_timer'))
        self.normal_mode_timer = rospy.Duration(rospy.get_param('~normal_mode_timer'))

        self.hotspot_timer = rospy.Timer(self.normal_mode_timer, self.__publish_hotspot_state_callback)

        self.manage_wifi_server = rospy.Service('/niryo_robot/wifi/manage', ManageWifi,
                                                self.__callback_manage_wifi)

        self.manage_ethernet_server = rospy.Service('/niryo_robot/ethernet/manage', ManageEthernet,
                                                    self.__callback_manage_ethernet)

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

    def __publish_hotspot_state_callback(self, _):
        conn_success, status = self.client.wifi_state()

        # In case of connection error we increase the timer interval to avoid useless requests
        if not conn_success:
            self.slow_mode = True
            self.hotspot_timer.shutdown()
            self.hotspot_timer = rospy.Timer(self.slow_mode_timer,
                                             self.__publish_hotspot_state_callback)
        elif self.slow_mode:
            self.slow_mode = False
            self.hotspot_timer.shutdown()
            self.hotspot_timer = rospy.Timer(self.normal_mode_timer,
                                             self.__publish_hotspot_state_callback)

        self.__publish_hotspot_state(conn_success, status)

    def __publish_hotspot_state(self, conn_success, status):
        msg = WifiStatus()
        if not conn_success:
            msg.status = msg.UNKNOWN
        elif not status['wlan0_state']:
            msg.status = msg.DISABLED
        elif status['hotspot_state']:
            msg.status = msg.HOTSPOT
        else:
            msg.status = msg.CONNECTED
        try:
            self.wifi_state_publisher.publish(msg)
        except rospy.ROSException:
            return

    def __callback_manage_wifi(self, msg):
        status, message = CommandStatus.SUCCESS, "Success"
        if msg.cmd == msg.HOTSPOT:
            conn_success, result = self.client.activate_hotspot()
        elif msg.cmd == msg.RESTART:
            conn_success, result = self.client.restart_wifi()
            message = result["detail"]
        elif msg.cmd == msg.DEACTIVATE:
            conn_success, result = self.client.deactivate_wifi()
            message = result["detail"]
        elif msg.cmd == msg.RECONNECT:
            conn_success, result = self.client.reconnect_last_wifi()
            status = CommandStatus.SUCCESS if result["success"] else CommandStatus.SYSTEM_API_CLIENT_COMMAND_FAILED
            message = result["detail"]
        else:
            return CommandStatus.SYSTEM_API_CLIENT_UNKNOWN_COMMAND, "Command {} not found".format(msg.cmd)

        if not conn_success:
            return CommandStatus.SYSTEM_API_CLIENT_REQUEST_FAILED, result
        if not result:
            return CommandStatus.SYSTEM_API_CLIENT_UNKNOWN_ERROR, "Failed to activate hotspot mode"

        conn_success, result = self.client.wifi_state()
        self.__publish_hotspot_state(conn_success, result)

        return status, message

    def __callback_manage_ethernet(self, req):
        status, message = CommandStatus.SUCCESS, "Success"
        if req.profile in [req.STATIC, req.AUTO]:
            conn_success, result = self.client.setup_ethernet(req.profile)
        elif req.profile == req.CUSTOM:
            conn_success, result = self.client.setup_ethernet(req.profile, req.ip, req.mask, req.gateway, req.dns)
            message = result["detail"]
        else:
            return CommandStatus.SYSTEM_API_CLIENT_UNKNOWN_COMMAND, "Command {} not found".format(req.profile)

        if not conn_success:
            return CommandStatus.SYSTEM_API_CLIENT_REQUEST_FAILED, result
        if not result:
            return CommandStatus.SYSTEM_API_CLIENT_UNKNOWN_ERROR, "Failed to change the ethernet profile"

        conn_success, result = self.client.wifi_state()
        self.__publish_hotspot_state(conn_success, result)

        return status, message


if __name__ == "__main__":
    rospy.init_node('niryo_robot_system_api_client', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    SystemApiClientNode()
    rospy.spin()
