#!/usr/bin/env python

import rospy

from niryo_robot_msgs.msg import CommandStatus
from std_msgs.msg import Bool

from niryo_robot_arm_commander.srv import JogShift, JogShiftRequest
from niryo_robot_msgs.srv import SetBool


class JogClient:
    def __init__(self):
        rospy.init_node('jog_client_node', anonymous=True)

        self.__jog_enabled = False
        self.__jog_enabled_subscriber = rospy.Subscriber('/niryo_robot/jog_interface/is_enabled',
                                                         Bool, self.__callback_subscriber_jog_enabled,
                                                         queue_size=1)

    def __callback_subscriber_jog_enabled(self, ros_data):
        self.__jog_enabled = ros_data.data

    def set_jog(self, set_bool):
        if set_bool == self.__jog_enabled:
            return CommandStatus.SUCCESS, "Already enable"
        rospy.wait_for_service('/niryo_robot/jog_interface/enable')
        try:
            enable_service = rospy.ServiceProxy('/niryo_robot/jog_interface/enable', SetBool)
            response = enable_service(set_bool)
        except rospy.ServiceException as e:
            raise Exception("Service call failed: {}".format(e))
        rospy.sleep(0.1)
        return response

    def ask_for_jog_shift(self, cmd, shift_values):
        init_time = rospy.get_time()
        if not self.__jog_enabled:
            self.set_jog(True)
        service_name = '/niryo_robot/jog_interface/jog_shift_commander'
        rospy.wait_for_service(service_name)
        try:
            jog_commander_service = rospy.ServiceProxy(service_name, JogShift)
            req = JogShiftRequest()
            req.cmd = cmd
            req.shift_values = shift_values
            response = jog_commander_service(req)
        except rospy.ServiceException as e:
            raise Exception("Service call failed: {}".format(e))
        rospy.sleep(0.15 - (rospy.get_time() - init_time))
        return response


if __name__ == "__main__":
    # Creating Client Object
    jc = JogClient()
    jc.set_jog(True)

    for sign in [1, -1]:
        for i in range(15):
            jc.ask_for_jog_shift(cmd=JogShiftRequest.JOINTS_SHIFT,
                                 shift_values=[sign * 0.05, 0.0, 0.0, 0.0, 0.0, 0.0])

    for sign in [1, -1]:
        for i in range(20):
            jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                 shift_values=[sign * 0.005, 0.0, 0.0, 0.0, 0.0, 0.0])

    for sign in [1, -1]:
        for i in range(15):
            jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                 shift_values=[0.0, 0.0, 0.0, 0.0, sign * 0.05, 0.0])

    for sign in [1, -1]:
        for i in range(15):
            jc.ask_for_jog_shift(cmd=JogShiftRequest.POSE_SHIFT,
                                 shift_values=[0.0, 0.0, 0.0, 0.0, 0.0, sign * 0.05])
