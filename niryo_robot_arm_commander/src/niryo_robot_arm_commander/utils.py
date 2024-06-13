#!/usr/bin/env python

# Lib
import copy
import numpy as np

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

# Messages
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

from niryo_robot_msgs.msg import RPY


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
    np_pose1 = np.array([
        p1.position.x,
        p1.position.y,
        p1.position.z,
        p1.orientation.x,
        p1.orientation.y,
        p1.orientation.z,
        p1.orientation.w
    ])
    np_pose2 = np.array([
        p2.position.x,
        p2.position.y,
        p2.position.z,
        p2.orientation.x,
        p2.orientation.y,
        p2.orientation.z,
        p2.orientation.w
    ])

    return np.linalg.norm(np_pose1 - np_pose2)


def dist_2_points(p1, p2):
    """
    :param p1:
    :type p1: Pose
    :param p2:
    :type p2: Pose

    :return: The distance (in meters) between the two poses
    :rtype: float
    """
    np_point1 = np.array([p1.x, p1.y, p1.z])
    np_point2 = np.array([p2.x, p2.y, p2.z])

    return np.linalg.norm(np_point1 - np_point2)


def angle_between_2_points(p1, p2):
    np_point1 = np.array([p1.x, p1.y, p1.z])
    np_point2 = np.array([p2.x, p2.y, p2.z])

    unit_vector_1 = np_point1 / np.linalg.norm(np_point1)
    unit_vector_2 = np_point2 / np.linalg.norm(np_point2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)

    return angle


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
    np_pose1 = np.array([
        p1.position.x,
        p1.position.y,
        p1.position.z,
        p1.orientation.x,
        p1.orientation.y,
        p1.orientation.z,
        p1.orientation.w
    ])
    np_pose2 = np.array([
        p2.position.x,
        p2.position.y,
        p2.position.z,
        p2.orientation.x,
        p2.orientation.y,
        p2.orientation.z,
        p2.orientation.w
    ])

    dist = np.linalg.norm(np_pose1 - np_pose2)

    return dist < 0.001


def invert_quat(q):
    q_inv = copy.deepcopy(q)
    if isinstance(q_inv, Quaternion):
        q_inv.w = -q.w
    elif isinstance(q_inv, list):
        q_inv[3] = -q[3]
    else:
        raise TypeError
    return q_inv


def diff_quat(q1, q2):
    q1_list = [q1.x, q1.y, q1.z, q1.w] if isinstance(q1, Quaternion) else q1
    q2_list = [q2.x, q2.y, q2.z, q2.w] if isinstance(q2, Quaternion) else q2

    q2_inv = invert_quat(q2_list)

    return quaternion_multiply(q1_list, q2_inv)


def get_orientation_from_angles(r, p, y):
    quaternion = quaternion_from_euler(r, p, y)
    orientation = Quaternion()
    orientation.x = quaternion[0]
    orientation.y = quaternion[1]
    orientation.z = quaternion[2]
    orientation.w = quaternion[3]
    return orientation


def get_rpy_from_quaternion(rot):
    euler = euler_from_quaternion(rot)
    # Force angles in [-PI, PI]
    for i, angle in enumerate(euler):
        if angle > np.pi:
            euler[i] = angle % (2 * np.pi) - 2 * np.pi
        elif angle < -np.pi:
            euler[i] = angle % (2 * np.pi)
    return euler


def quaternion_to_list(quat):
    return [quat.x, quat.y, quat.z, quat.w]


def vector3_to_list(vect):
    return [vect.x, vect.y, vect.z]


def list_to_vector3(list_):
    return Vector3(x=list_[0], y=list_[1], z=list_[2])


def list_to_rpy(list_):
    return RPY(roll=list_[0], pitch=list_[1], yaw=list_[2])
