#!/usr/bin/env python

import rospy
import logging

from niryo_robot_user_interface.tcp_server import TcpServer

from std_msgs.msg import Bool
from niryo_robot_user_interface.msg import ConnectionState

from niryo_robot_sound.srv import PlaySound, PlaySoundRequest
from niryo_robot_database.srv import GetSettings


class UserInterface:

    def __init__(self):
        rospy.logdebug("User Interface Node - Entering in Init")

        ip_address = rospy.get_param("~ip_address", '')
        port = rospy.get_param("~tcp_port", 40001)

        self.__is_client_connected_pub = rospy.Publisher('~is_client_connected', Bool, latch=True, queue_size=10)
        self.__is_client_connected_pub.publish(False)

        self.__tcp_server = TcpServer(ip_address=ip_address,
                                      port=port,
                                      on_client_connection_cb=self.on_client_connection,
                                      on_client_disconnection_cb=self.on_client_disconnection).start()

        rospy.wait_for_service('/niryo_robot_database/settings/get', 20)
        self.__get_setting = rospy.ServiceProxy('/niryo_robot_database/settings/get', GetSettings)
        self.__uuid = self.__get_setting('rpi_uuid').value

        self.__play_overlay_service = rospy.ServiceProxy('/niryo_robot_sound/overlay', PlaySound)

        self.__niryo_studio_connections = {}
        rospy.Subscriber('~niryo_studio_connection', ConnectionState, self.__niryo_studio_connection_callback)
        rospy.Timer(rospy.Duration(3), self.__check_niryo_studio_connections)

        self.__robot_connection_pub = rospy.Publisher('~robot_connection', ConnectionState, queue_size=10)
        self.__robot_connection_timer = rospy.Timer(
            rospy.Duration(1), lambda _: self.__robot_connection_pub.publish(self.__uuid, ConnectionState.ok))

        if self.__tcp_server is not None:
            # Set a bool to mentioned this node is initialized
            rospy.set_param('~initialized', True)
            rospy.loginfo("User Interface Node - Started")
        else:
            rospy.set_param('~initialized', False)
            rospy.logerr("User Interface Node - Not correctly Started")

    def __niryo_studio_connection_callback(self, msg):
        if msg.state == ConnectionState.connection:
            self.niryo_studio_connected(msg.uuid)
        elif msg.state == ConnectionState.ok:
            self.__niryo_studio_connections[msg.uuid] = rospy.Time.now()
        elif msg.state == ConnectionState.close:
            self.niryo_studio_disconnected(msg.uuid)

    def __check_niryo_studio_connections(self, event):
        for uuid in list(self.__niryo_studio_connections.keys()):
            if event.current_real - self.__niryo_studio_connections[uuid] > rospy.Duration(3):
                self.niryo_studio_disconnected(uuid)

    def niryo_studio_connected(self, uuid):
        self.__niryo_studio_connections[uuid] = rospy.Time.now()
        self.__play_overlay_service(PlaySoundRequest(sound_name='connected.wav'))

    def niryo_studio_disconnected(self, uuid):
        self.__play_overlay_service(PlaySoundRequest(sound_name='disconnected.wav'))
        del self.__niryo_studio_connections[uuid]

    def shutdown(self):
        if self.__tcp_server is not None:
            self.__tcp_server.quit()
            self.__robot_connection_timer.shutdown()
        self.__robot_connection_pub.publish(self.__uuid, ConnectionState.close)

    def on_client_connection(self):
        self.__is_client_connected_pub.publish(True)

    def on_client_disconnection(self):
        self.__is_client_connected_pub.publish(False)


if __name__ == '__main__':
    rospy.init_node('niryo_robot_user_interface', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        ui = UserInterface()
        rospy.on_shutdown(ui.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
