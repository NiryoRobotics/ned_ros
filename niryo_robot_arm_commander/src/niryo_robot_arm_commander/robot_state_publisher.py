#!/usr/bin/env python

from copy import deepcopy

import numpy
import rospy
from tf import LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np

from niryo_robot_msgs.msg import RobotState
from niryo_robot_poses_handlers.transform_functions import convert_dh_convention_to_legacy_rpy

from .utils import quaternion_to_list, vector3_to_list, list_to_vector3, get_orientation_from_angles, list_to_rpy


class StatePublisher(object):
    """
    This object read Transformation Publisher and Publish the RobotState
     in the Topic '/niryo_robot/robot_state' at a certain rate
    """

    def __init__(self, arm_state):
        self.__arm_state = arm_state

        self.__last_robot_state = RobotState()
        self.__last_stamp = rospy.Time.now()

        self.__transform_handler = self.__arm_state.transform_handler

        # State publisher
        self.__robot_state_publisher = rospy.Publisher('/niryo_robot/robot_state', RobotState, queue_size=5)
        self.__robot_state_v2_publisher = rospy.Publisher('/niryo_robot/robot_state_v2', RobotState, queue_size=5)

        # Get params from rosparams
        rate_publish_state = rospy.get_param("/niryo_robot/robot_state/rate_publish_state")
        rospy.logdebug("StatePublisher.init - rate_publish_state: %s", rate_publish_state)

        rospy.Timer(rospy.Duration(1.0 / rate_publish_state), self.__publish_states)

    def __get_robot_state(self):
        try:
            t = self.__transform_handler.lookup_transform('base_link', 'TCP', rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if isinstance(e, ExtrapolationException):
                print(e)
            self.__transform_handler.set_empty_tcp_to_ee_link_transform("tool_link")
            rospy.loginfo_throttle(1, "State Publisher - Failed to get TF base_link -> TCP")
            return

        robot_state = RobotState()
        robot_state.position = t.transform.translation
        robot_state.orientation = t.transform.rotation
        quaternion_list = quaternion_to_list(t.transform.rotation)
        robot_state.rpy = list_to_rpy(euler_from_quaternion(quaternion_list))

        dt = (t.header.stamp - self.__last_stamp).to_sec()
        if dt == 0:
            return self.__last_robot_state

        position_array = numpy.array(vector3_to_list(t.transform.translation))
        last_position_array = numpy.array(vector3_to_list(self.__last_robot_state.position))
        linear_velocity = [round(x, 4) for x in (position_array - last_position_array) / dt]

        orientation_array = numpy.array(quaternion_list)
        last_orientation_array = numpy.array(quaternion_to_list(self.__last_robot_state.orientation))
        angular_velocity = [round(x, 4) for x in (orientation_array - last_orientation_array) / dt]

        robot_state.twist.linear = list_to_vector3(linear_velocity)
        robot_state.twist.angular = list_to_vector3(angular_velocity)
        robot_state.tcp_speed = np.linalg.norm(linear_velocity)

        self.__last_stamp = t.header.stamp
        self.__last_robot_state = deepcopy(robot_state)
        return robot_state

    def __publish_states(self, _):
        robot_state = self.__get_robot_state()

        try:
            self.__robot_state_v2_publisher.publish(robot_state)
        except Exception:
            return

        rpy_v1 = convert_dh_convention_to_legacy_rpy(robot_state.rpy.roll, robot_state.rpy.pitch, robot_state.rpy.yaw)
        robot_state.rpy = list_to_rpy(rpy_v1)
        robot_state.orientation = get_orientation_from_angles(*rpy_v1)

        try:
            self.__robot_state_publisher.publish(robot_state)
        except Exception:
            return
