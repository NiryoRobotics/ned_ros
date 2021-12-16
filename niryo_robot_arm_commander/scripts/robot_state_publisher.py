#!/usr/bin/env python

import rospy
from tf import LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import pi
import numpy as np

from geometry_msgs.msg import Quaternion, Twist, Vector3
from niryo_robot_msgs.msg import RobotState

from niryo_robot_arm_commander.utils import diff_quat


class StatePublisher(object):
    """
    This object read Transformation Publisher and Publish the RobotState
     in the Topic '/niryo_robot/robot_state' at a certain rate
    """

    def __init__(self, arm_state):
        self.__arm_state = arm_state

        # Tf listener (position + rpy) of end effector tool
        self.__position = [0, 0, 0]
        self.__quaternion = [0, 0, 0, 0]
        self.__rpy = [0, 0, 0]
        self.__stamp = rospy.Time.now()
        self.__twist = Twist()
        self.__tcp_speed = 0

        self.__transform_handler = self.__arm_state.transform_handler

        # State publisher
        self.__robot_state_publisher = rospy.Publisher('/niryo_robot/robot_state', RobotState, queue_size=5)

        # Get params from rosparams
        rate_publish_state = rospy.get_param("/niryo_robot/robot_state/rate_publish_state")
        rospy.logdebug("StatePublisher.init - rate_publish_state: %s", rate_publish_state)

        rospy.Timer(rospy.Duration(1.0 / rate_publish_state), self.__publish_state)

    def __update_ee_link_pose(self):
        try:
            t = self.__transform_handler.lookup_transform('base_link', 'TCP', rospy.Time(0))

            pos = self.vector3_to_list(t.transform.translation)
            quat = self.quaternion_to_list(t.transform.rotation)

            dt = (t.header.stamp - self.__stamp).to_sec()
            if dt > 0:
                lin_vel = [round(x, 4) for x in (np.array(pos) - np.array(self.__position)) / dt]
                rot_vel = [round(x, 4) for x in
                           euler_from_quaternion(np.array(diff_quat(quat, self.__quaternion)) / dt)]

                self.__twist.linear = Vector3(*lin_vel)
                self.__twist.angular = Vector3(*rot_vel)
                self.__tcp_speed = np.linalg.norm(lin_vel)

            self.__stamp = t.header.stamp
            self.__position = pos
            self.__quaternion = quat
            self.__rpy = euler_from_quaternion(self.__quaternion)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.__transform_handler.set_empty_tcp_to_ee_link_transform("tool_link")
            rospy.loginfo_throttle(1, "State Publisher - Failed to get TF base_link -> TCP")

    def __publish_state(self, _):
        self.__update_ee_link_pose()

        msg = RobotState()
        msg.position.x = self.__position[0]
        msg.position.y = self.__position[1]
        msg.position.z = self.__position[2]
        msg.rpy.roll = self.__rpy[0]
        msg.rpy.pitch = self.__rpy[1]
        msg.rpy.yaw = self.__rpy[2]
        msg.orientation.x = self.__quaternion[0]
        msg.orientation.y = self.__quaternion[1]
        msg.orientation.z = self.__quaternion[2]
        msg.orientation.w = self.__quaternion[3]
        msg.twist = self.__twist
        msg.tcp_speed = self.__tcp_speed

        self.__arm_state.robot_state = msg
        try:
            self.__robot_state_publisher.publish(msg)
        except rospy.ROSException:
            return

    @staticmethod
    def get_orientation_from_angles(r, p, y):
        quaternion = quaternion_from_euler(r, p, y)
        orientation = Quaternion()
        orientation.x = quaternion[0]
        orientation.y = quaternion[1]
        orientation.z = quaternion[2]
        orientation.w = quaternion[3]
        return orientation

    @staticmethod
    def get_rpy_from_quaternion(rot):
        euler = euler_from_quaternion(rot)
        # Force angles in [-PI, PI]
        for i, angle in enumerate(euler):
            if angle > pi:
                euler[i] = angle % (2 * pi) - 2 * pi
            elif angle < -pi:
                euler[i] = angle % (2 * pi)
        return euler

    @staticmethod
    def quaternion_to_list(quat):
        return [quat.x, quat.y, quat.z, quat.w]

    @staticmethod
    def vector3_to_list(vect):
        return [vect.x, vect.y, vect.z]
