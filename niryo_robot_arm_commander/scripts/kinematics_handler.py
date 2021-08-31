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

from niryo_robot_msgs.msg import RobotState, RPY

# Services
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from niryo_robot_arm_commander.srv import GetFK, GetIK


class KinematicsHandler:
    """
    Object which handle the arm kinematics functions.
    """

    def __init__(self, arm_move_group, transform_handler):
        self.__joints = None
        self.__joints_name = rospy.get_param('~joint_names')
        rospy.Subscriber('/joint_states', JointState, self.__callback_sub_joint_states)

        # Get Arm MoveGroupCommander
        self.__arm = arm_move_group
        self.__transform_handler = transform_handler

        # - CALLABLE SERVICES
        # Kinematics
        rospy.Service('/niryo_robot/kinematics/forward', GetFK, self.__callback_get_forward_kinematics)
        rospy.Service('/niryo_robot/kinematics/inverse', GetIK, self.__callback_get_inverse_kinematics)

    # -- Callbacks
    def __callback_sub_joint_states(self, joint_states):
        self.__joints = list(joint_states.position[:6])

    def __callback_get_forward_kinematics(self, req):
        return self.get_forward_kinematics(joints=req.joints)

    def __callback_get_inverse_kinematics(self, req):
        pose = req.pose
        if pose.orientation == Quaternion():
            angles = pose.rpy
            qx, qy, qz, qw = quaternion_from_euler(angles.roll, angles.pitch, angles.yaw)
            pose.orientation = Quaternion(qx, qy, qz, qw)
        pose = Pose(pose.position, pose.orientation)
        success, joints = self.get_inverse_kinematics(pose=pose)
        return success, joints

    def get_forward_kinematics(self, joints):
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
            return RobotState()
        try:
            moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
            fk_link = ['base_link', 'tool_link']
            header = Header(0, rospy.Time.now(), "world")
            rs = RobotStateMoveIt()
            rs.joint_state.name = self.__joints_name
            rs.joint_state.position = joints
            response = moveit_fk(header, fk_link, rs)
        except rospy.ServiceException as e:
            rospy.logerr("Arm commander - Failed to get FK : " + str(e))
            return RobotState()

        pose = self.__transform_handler.ee_link_to_tcp_pose_target(response.pose_stamped[1].pose, "tool_link")

        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rpy = euler_from_quaternion(quaternion)
        quaternion = (round(quaternion[0], 3), round(quaternion[1], 3), round(quaternion[2], 3),
                      round(quaternion[3], 3))
        point = (round(pose.position.x, 3),
                 round(pose.position.y, 3),
                 round(pose.position.z, 3))

        return RobotState(Point(*point), RPY(*rpy), Quaternion(*quaternion))

    def get_inverse_kinematics(self, pose):
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

            req.robot_state.joint_state.name = self.__joints_name
            req.robot_state.joint_state.position = self.__joints
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
