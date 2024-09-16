#!/usr/bin/env python
import rospy
import logging

from niryo_robot_utils import sentry_init

from niryo_robot_status.robot_status_handler import RobotStatusHandler


class RobotStatus:

    def __init__(self):
        self.__robot_status_handler = RobotStatusHandler()


if __name__ == '__main__':
    sentry_init()

    rospy.init_node('niryo_robot_status', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        node = RobotStatus()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
