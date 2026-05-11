#!/usr/bin/env python

import rospy
import logging
from niryo_robot_modbus.ModbusServer import ModbusServer
from niryo_robot_database.msg import Setting
from niryo_robot_database.srv import GetSettings, SetSettings
from niryo_robot_utils import async_init
from threading import Lock

from niryo_robot_utils import sentry_init


class ModbusServerNode:

    def __init__(self):

        self.__modbus_server = None
        self._server_lock = Lock()
        self.name = "modbus_port"

        # Retrieved parameters
        self.__modbus_server_address = rospy.get_param("~server_address")
        self.__modbus_server_port = rospy.get_param("~server_port")

        rospy.logdebug("ModbusServerNode.Init - server_address: %s", self.__modbus_server_address)

        rospy.Subscriber('/niryo_robot_database/setting_update', Setting, self._setting_update_callback)
        async_init.PromiseServiceProxy('/niryo_robot_database/settings/get',
                                       GetSettings,
                                       self._on_get_settings_available)
        self.__set_setting_service = rospy.ServiceProxy('/niryo_robot_database/settings/set', SetSettings)
        # Create Modbus
        with self._server_lock:
            rospy.loginfo("ModbusServerNode.Init - server_port: %s", self.__modbus_server_port)

            try:
                self.__modbus_server = ModbusServer(self.__modbus_server_address, self.__modbus_server_port)
            except Exception as e:
                rospy.logerr(f"Modbus Node - Failed to instantiate server: {e}")
            if self.__modbus_server:
                # Stop on ROS shutdown
                rospy.on_shutdown(self.__modbus_server.stop)

                # Start server
                self.__modbus_server.start()

                rospy.loginfo("Modbus Node - Started")

            else:
                rospy.logerr("Modbus Node - Not Correctly Started")

    def _setting_update_callback(self, req):
        if req.name == self.name:
            self._update_port(req.value)

    def _on_get_settings_available(self, get_settings_proxy):
        response = get_settings_proxy(self.name)
        if response.status < 0:
            self.__set_setting_service(self.name, str(self.__modbus_server_port), 'str')
            rospy.logwarn(f'The modbus port was not found in the database. Defaulting to "{self.__modbus_server_port}"')
            return

        self._update_port(response.value)

    def _update_port(self, new_port):
        with self._server_lock:
            if self.__modbus_server_port != new_port:
                if self.__modbus_server:
                    self.__modbus_server.change_port(new_port)
                self.__modbus_server_port = new_port


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
