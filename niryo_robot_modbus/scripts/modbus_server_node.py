#!/usr/bin/env python

import rospy
from modbus_server import ModbusServer


class ModbusServerNode:
    def __init__(self):
        # Retrieved parameters    
        self.__modbus_server_address = rospy.get_param("~server_address")
        self.__modbus_server_port = rospy.get_param("~server_port")

        # Create Modbus server
        self.__modbus_server = ModbusServer(self.__modbus_server_address, self.__modbus_server_port)

        # Stop on ROS shutdown
        rospy.on_shutdown(self.__modbus_server.stop)

        # Start server
        self.__modbus_server.start()

        rospy.loginfo("Modbus Node - Started")


if __name__ == '__main__':
    rospy.init_node('niryo_robot_modbus')
    modbus_server_node = ModbusServerNode()
    # Loop until ros shutdown
    rospy.spin()
