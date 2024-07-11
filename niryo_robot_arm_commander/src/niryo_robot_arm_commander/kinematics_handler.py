#!/usr/bin/env python

# Lib
import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import RobotState as RobotStateMoveIt
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from niryo_robot_msgs.msg import RobotState, RPY, CommandStatus

# Services
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from niryo_robot_arm_commander.srv import GetFK, GetIK

# Utils
from niryo_robot_poses_handlers.transform_functions import convert_dh_convention_to_legacy_rpy


class KinematicsHandler:
    """
    Object which handle the arm kinematics functions.
    """

    def __init__(self, arm_state):
        self.__arm_state = arm_state

        # Get Arm MoveGroupCommander
        self.__arm = self.__arm_state.arm
        self.__transform_handler = self.__arm_state.transform_handler

        # - CALLABLE SERVICES
        # Kinematics
        rospy.Service('/niryo_robot/kinematics/forward', GetFK, self.__callback_get_forward_kinematics)
        rospy.Service('/niryo_robot/kinematics/inverse', GetIK, self.__callback_get_inverse_kinematics)

        rospy.Service('/niryo_robot/kinematics/forward_v2', GetFK, self.__callback_get_forward_kinematics_v2)
        rospy.Service('/niryo_robot/kinematics/inverse_v2', GetIK, self.__callback_get_inverse_kinematics_v2)

    # -- Callbacks
    def __callback_get_forward_kinematics(self, req):
        forward_kinematics = self.get_forward_kinematics(joints=req.joints)
        if forward_kinematics is None:
            return (CommandStatus.FORWARD_KINEMATICS_FAILURE,
                    'An error occurred while computing forward kinematics.',
                    RobotState())
        return CommandStatus.SUCCESS, 'Successfully computed forward kinematic', forward_kinematics

    def __callback_get_forward_kinematics_v2(self, req):
        forward_kinematics = self.get_forward_kinematics_v2(joints=req.joints)
        if forward_kinematics is None:
            return (CommandStatus.FORWARD_KINEMATICS_FAILURE,
                    'An error occurred while computing forward kinematics.',
                    RobotState())
        return CommandStatus.SUCCESS, 'Successfully computed forward kinematic', forward_kinematics

    def __callback_get_inverse_kinematics(self, req):
        pose = req.pose
        if pose.orientation == Quaternion():
            angles = pose.rpy
            qx, qy, qz, qw = quaternion_from_euler(angles.roll, angles.pitch, angles.yaw)
            pose.orientation = Quaternion(qx, qy, qz, qw)
        pose = Pose(pose.position, pose.orientation)
        success, joints = self.get_inverse_kinematics(pose=pose)
        if not success:
            return (CommandStatus.INVERT_KINEMATICS_FAILURE,
                    'An error occurred while computing the inverse kinematic',
                    joints)
        return CommandStatus.SUCCESS, '', joints

    def __callback_get_inverse_kinematics_v2(self, req):
        pose = req.pose
        if pose.orientation == Quaternion():
            angles = pose.rpy
            qx, qy, qz, qw = quaternion_from_euler(angles.roll, angles.pitch, angles.yaw)
            pose.orientation = Quaternion(qx, qy, qz, qw)
        pose = Pose(pose.position, pose.orientation)
        success, joints = self.get_inverse_kinematics_v2(pose=pose)
        if not success:
            return (CommandStatus.INVERT_KINEMATICS_FAILURE,
                    'An error occurred while computing the inverse kinematic',
                    joints)
        return CommandStatus.SUCCESS, '', joints

    def get_forward_kinematics(self, joints):
        """
        Get forward kinematic from joints and tranform the pose of the returned Robotstate
        to match former Niryo's TCP convention
        :param joints: list of joints value
        :type joints: Pose
        :return: A RobotState object
        """
        robot_state = self.get_forward_kinematics_v2(joints)

        rpy_v1 = convert_dh_convention_to_legacy_rpy(robot_state.rpy.roll, robot_state.rpy.pitch, robot_state.rpy.yaw)
        quaternion_v1 = quaternion_from_euler(*rpy_v1)

        return RobotState(position=Point(robot_state.position.x, robot_state.position.y, robot_state.position.z),
                          rpy=RPY(*rpy_v1),
                          orientation=Quaternion(*quaternion_v1))

    def get_forward_kinematics_v2(self, joints):
        """
        Get forward kinematic from joints
        :param joints: list of joints value
        :type joints: Pose
        :return: A RobotState object
        """
        try:
            rospy.wait_for_service('compute_fk', 2)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Arm commander - Impossible to connect to FK service : " + str(e))
            return None
        try:
            moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
            fk_link = ['base_link', 'tool_link']
            header = Header(0, rospy.Time.now(), "world")
            rs = RobotStateMoveIt()
            rs.joint_state.name = self.__arm_state.joints_name
            rs.joint_state.position = joints
            response = moveit_fk(header, fk_link, rs)
        except rospy.ServiceException as e:
            rospy.logerr("Arm commander - Failed to get FK : " + str(e))
            return None

        pose = self.__transform_handler.ee_link_to_tcp_pose_target(response.pose_stamped[1].pose, "tool_link")

        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rpy = euler_from_quaternion(quaternion)
        quaternion = (round(quaternion[0], 3),
                      round(quaternion[1], 3),
                      round(quaternion[2], 3),
                      round(quaternion[3], 3))
        point = (round(pose.position.x, 3), round(pose.position.y, 3), round(pose.position.z, 3))

        return RobotState(position=Point(*point), rpy=RPY(*rpy), orientation=Quaternion(*quaternion))

    def get_inverse_kinematics(self, pose):
        """
        Transform the pose to match former Niryo's TCP convention and get inverse kinematic from pose
        :param pose: pose
        :type geometry_msgs/Pose: Pose
        :return: success, a list of joints value
        """
        rpy = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        rpy_legacy = convert_dh_convention_to_legacy_rpy(*rpy)

        qx, qy, qz, qw = quaternion_from_euler(*rpy_legacy)

        legacy_pose = Pose(position=pose.position, orientation=Quaternion(qx, qy, qz, qw))

        return self.get_inverse_kinematics_v2(legacy_pose)

    def get_inverse_kinematics_v2(self, pose):
        """
        Get inverse kinematic from pose
        :param pose: pose
        :type geometry_msgs/Pose: Pose
        :return: success, a list of joints value
        """
        try:
            rospy.wait_for_service('compute_ik', 2)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Arm commander - Impossible to connect to IK service : " + str(e))
            return False, []
        try:
            tool_link_pose = self.__transform_handler.tcp_to_ee_link_pose_target(pose, "tool_link")

            moveit_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
            req = PositionIKRequest()
            req.group_name = self.__arm.get_name()
            req.ik_link_name = self.__arm.get_end_effector_link()

            req.robot_state.joint_state.name = self.__arm_state.joints_name
            req.robot_state.joint_state.position = self.__arm_state.joint_states
            req.pose_stamped.pose = tool_link_pose

            response = moveit_ik(req)
        except rospy.ServiceException as e:
            rospy.logerr("Arm commander - Service call failed: {}".format(e))
            return False, []
        if response.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
            rospy.logerr_throttle(0.5, "Arm commander - MoveIt didn't find an IK solution")
            return False, []
        elif response.error_code.val != MoveItErrorCodes.SUCCESS:
            rospy.logerr("Arm commander - IK Failed : code error {}".format(response.error_code.val))
            return False, []
        return True, list(response.solution.joint_state.position[:6])
