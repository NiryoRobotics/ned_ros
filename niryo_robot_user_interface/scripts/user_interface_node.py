#!/usr/bin/env python

import rospy

from niryo_robot_user_interface.tcp_server import TcpServer


class UserInterface:

    def __init__(self):
        rospy.logdebug("User Interface Node - Entering in Init")

        self.__tcp_server = TcpServer().start()

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("User Interface Node - Started")

    def shutdown(self):
        self.__tcp_server.quit()


if __name__ == '__main__':
    rospy.init_node('niryo_robot_user_interface', anonymous=False, log_level=rospy.INFO)
    try:
        ui = UserInterface()
        rospy.on_shutdown(ui.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
