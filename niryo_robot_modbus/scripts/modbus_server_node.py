#!/usr/bin/env python

import rospy
import logging
from niryo_robot_modbus.ModbusServer import ModbusServer

from niryo_robot_utils import sentry_init


class ModbusServerNode:

    def __init__(self):
        # Retrieved parameters
        self.__modbus_server_address = rospy.get_param("~server_address")
        self.__modbus_server_port = rospy.get_param("~server_port")

        rospy.logdebug("ModbusServerNode.Init - server_address: %s", self.__modbus_server_address)
        rospy.logdebug("ModbusServerNode.Init - server_port: %s", self.__modbus_server_port)

        # Create Modbus server
        self.__modbus_server = ModbusServer(self.__modbus_server_address, self.__modbus_server_port)

        if self.__modbus_server is not None:
            # Stop on ROS shutdown
            rospy.on_shutdown(self.__modbus_server.stop)

            # Start server
            self.__modbus_server.start()

            rospy.loginfo("Modbus Node - Started")

        else:
            rospy.loginfo("Modbus Node - Not Correctly Started")


if __name__ == '__main__':
    sentry_init()

    rospy.init_node('niryo_robot_modbus', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    modbus_server_node = ModbusServerNode()
    # Loop until ros shutdown
    rospy.spin()
