#!/usr/bin/env python

import rospy
from urdf_parser_py.urdf import URDF

from math import sqrt

from .command_enums import ArmCommanderException

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from geometry_msgs.msg import Point
from niryo_robot_msgs.msg import RPY
from niryo_robot_arm_commander.msg import JointLimits

# Services
from niryo_robot_arm_commander.srv import GetJointLimits, GetJointLimitsResponse
from moveit_msgs.srv import GetStateValidity


class ArmParametersValidator:
    """
    Object which allows to validate arm movement
    """

    def __init__(self, validation_consts):
        self.validation_consts = validation_consts

        self.joint_name_list = rospy.get_param('~joint_names')

        self.joints_limits_dict = {}
        self.joints_limits = []

        self.joints_limits_from_urdf()

        self.joint_limits_service = rospy.Service('~get_joints_limit', GetJointLimits, self.__callback_get_joint_limits)

        # Check joint validity service (used for self collisions checking)
        self.check_state_validity = rospy.ServiceProxy('check_state_validity', GetStateValidity)

    def __callback_get_joint_limits(self, _req):
        resp = GetJointLimitsResponse()
        resp.joint_limits = []
        for joint_name in self.joint_name_list:
            limit = JointLimits(joint_name,
                                self.joints_limits_dict[joint_name].lower,
                                self.joints_limits_dict[joint_name].upper)
            resp.joint_limits.append(limit)
        return resp

    def joints_limits_from_urdf(self):
        robot_urdf = URDF.from_parameter_server()

        self.joints_limits = []
        for joint in robot_urdf.joints:
            if joint.name in self.joint_name_list:
                self.joints_limits.append(joint.limit)
                self.joints_limits_dict[joint.name] = joint.limit
                rospy.set_param("/niryo_robot/robot_command_validation/joint_limits/{}".format(joint.name), {
                    "min": joint.limit.lower, "max": joint.limit.upper
                })

        return self.joints_limits

    def get_joints_limits(self):
        return self.joints_limits

    def validate_trajectory(self, trajectory):
        rospy.loginfo("Checking trajectory validity")
        count = 0
        for joint in trajectory.points:
            try:
                self.validate_joints(joint.positions)
            except ArmCommanderException as e:
                raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS,
                                            "Waypoint number {} has invalid parameters: {}".format(count,
                                                                                                   e.message)) from e
            count += 1

    def validate_joints(self, joint_array):
        if len(joint_array) != len(self.joints_limits):
            raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS,
                                        "Joint array must have {} joints".format(len(self.joints_limits)))

        for joint_index, joint_cmd in enumerate(joint_array):
            if self.joints_limits[joint_index] and not self.joints_limits[joint_index].lower <= round(joint_cmd, 3) <= \
                                                       self.joints_limits[joint_index].upper:
                raise ArmCommanderException(
                    CommandStatus.INVALID_PARAMETERS,
                    "joint_{} not in range ({}, {}), {} provided".format(joint_index + 1,
                                                                         self.joints_limits[joint_index].lower,
                                                                         self.joints_limits[joint_index].upper,
                                                                         round(joint_cmd, 3)))

    def validate_position(self, position):
        if isinstance(position, Point):
            x = position.x
            y = position.y
            z = position.z
        else:
            x, y, z = position

        v = self.validation_consts['position_limits']

        if x < v['x']['min'] or x > v['x']['max']:
            raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS,
                                        "x not in range ( {} , {} )".format(v['x']['min'], v['x']['max']))
        if y < v['y']['min'] or y > v['y']['max']:
            raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS,
                                        "y not in range ( {} , {} )".format(v['y']['min'], v['y']['max']))
        if z < v['z']['min'] or z > v['z']['max']:
            raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS,
                                        "z not in range ( {} , {} )".format(v['z']['min'], v['z']['max']))

    def validate_orientation(self, orientation):
        if isinstance(orientation, RPY):
            roll = orientation.roll
            pitch = orientation.pitch
            yaw = orientation.yaw
        else:
            roll = orientation[0]
            pitch = orientation[1]
            yaw = orientation[2]

        v = self.validation_consts['rpy_limits']

        if roll < v['roll']['min'] or roll > v['roll']['max']:
            raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS,
                                        "roll not in range ( {} , {} )".format(v['roll']['min'], v['roll']['max']))

        if pitch < v['pitch']['min'] or pitch > v['pitch']['max']:
            raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS,
                                        "pitch not in range ( {} , {} )".format(v['pitch']['min'], v['pitch']['max']))

        if yaw < v['yaw']['min'] or yaw > v['yaw']['max']:
            raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS,
                                        "yaw not in range ( {} , {} )".format(v['yaw']['min'], v['yaw']['max']))

    @staticmethod
    def validate_orientation_quaternion(quat):
        norm = quat.x**2 + quat.y**2 + quat.z**2 + quat.w**2
        norm = sqrt(norm)
        if abs(norm - 1.0) > 0.001:
            raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS, "Quaternion is not normalised.")

    @staticmethod
    def validate_shift_pose(shift):
        if shift.axis_number not in [0, 1, 2, 3, 4, 5]:
            raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS, "shift axis number not in [0,1,2,3,4,5]")
        if shift.value < -1.0 or shift.value > 1.0:
            raise ArmCommanderException(CommandStatus.INVALID_PARAMETERS, "shift value can't be null and < -1 or > 1")
