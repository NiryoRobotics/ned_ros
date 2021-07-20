#!/usr/bin/env python

import rospy
import logging

from niryo_robot_user_interface.tcp_server import TcpServer


class UserInterface:

    def __init__(self):
        rospy.logdebug("User Interface Node - Entering in Init")

        ip_address = rospy.get_param("~ip_address", '')
        port = rospy.get_param("~tcp_port", 40001)

        self.__tcp_server = TcpServer(ip_address, port).start()
        if self.__tcp_server is not None:
            # Set a bool to mentioned this node is initialized
            rospy.set_param('~initialized', True)
            rospy.loginfo("User Interface Node - Started")
        else:
            rospy.set_param('~initialized', False)
            rospy.logerr("User Interface Node - Not correctly Started")


    def shutdown(self):
        if self.__tcp_server is not None:
            self.__tcp_server.quit()


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
