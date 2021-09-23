#!/usr/bin/env python

# Lib
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np

# Messages
from geometry_msgs.msg import Pose, Point, Quaternion


# - Useful functions
def pose_to_list(pose):
    """
    Take a ROS Pose and return it as a Python list [x, y, z, roll, pitch, yaw]

    :param pose: a Pose
    :type pose: Pose
    :return: The pose as a list
    :rtype: list[float]
    """
    x, y, z = pose.position.x, pose.position.y, pose.position.z
    roll, pitch, yaw = euler_from_quaternion(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z,
         pose.orientation.w])
    return [x, y, z, roll, pitch, yaw]


def list_to_pose(list_):
    """
    Take a Pose as a Python list [x, y, z, roll, pitch, yaw] and return a ROS Pose

    :param list_: a pose in a list
    :type list_: list[float]
    :return: a ROS Pose
    :rtype: Pose
    """
    x, y, z = list_[:3]
    q_x, q_y, q_z, q_w = quaternion_from_euler(*list_[3:])

    return Pose(Point(x, y, z), Quaternion(q_x, q_y, q_z, q_w))


def dist_2_poses(p1, p2):
    """
    :param p1:
    :type p1: Pose
    :param p2:
    :type p2: Pose

    :return: The distance (in meters) between the two poses
    :rtype: float
    """
    np_pose1 = np.array([p1.position.x, p1.position.y, p1.position.z, p1.orientation.x, p1.orientation.y,
                         p1.orientation.z, p1.orientation.w])
    np_pose2 = np.array([p2.position.x, p2.position.y, p2.position.z, p2.orientation.x, p2.orientation.y,
                         p2.orientation.z, p2.orientation.w])

    return np.linalg.norm(np_pose1 - np_pose2)


def poses_too_close(p1, p2):
    """
    Check that distance between p1 and p2 is bigger than the minimal required distance for a move.

    :param p1:
    :type p1: Pose
    :param p2:
    :type p2: Pose

    :return: True if the distance between p1 and p2 is smaller than 1mm
    :rtype: bool
    """
    np_pose1 = np.array([p1.position.x, p1.position.y, p1.position.z, p1.orientation.x, p1.orientation.y,
                         p1.orientation.z, p1.orientation.w])
    np_pose2 = np.array([p2.position.x, p2.position.y, p2.position.z, p2.orientation.x, p2.orientation.y,
                         p2.orientation.z, p2.orientation.w])

    dist = np.linalg.norm(np_pose1 - np_pose2)

    return dist < 0.001
