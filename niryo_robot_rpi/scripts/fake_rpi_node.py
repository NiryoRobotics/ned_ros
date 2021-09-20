#!/usr/bin/env python

import rospy
import logging
from std_msgs.msg import Int32, Bool
from io_panel import FakeIOPanel, FakeDigitalIOPanel


class FakeNiryoButton:
    def __init__(self):
        rospy.logdebug("FakeNiryoButton - Entering in Init")

        # Publisher used to send info to Niryo Studio, so the user can add a move block
        # by pressing the button
        self.save_point_publisher = rospy.Publisher(
            "/niryo_robot/blockly/save_current_point", Int32, queue_size=10)

        self.__button_state_publisher = rospy.Publisher(
            "/niryo_robot/rpi/is_button_pressed", Bool, latch=True, queue_size=1)

        self.__button_state_publisher.publish(False)


class NiryoFakeRpi(object):
    def __init__(self):
        self.niryo_robot_fake_button = FakeNiryoButton()
        if rospy.get_param("~hardware_version") == "ned2":
            self.digital_io_panel = FakeIOPanel()
        else:
            self.digital_io_panel = FakeDigitalIOPanel()


if __name__ == '__main__':
    rospy.init_node('niryo_robot_rpi', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    NiryoFakeRpi()
    rospy.spin()
