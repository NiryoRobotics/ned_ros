#!/usr/bin/env python

# Lib
import rospy
import moveit_commander
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import copy
import threading

import math
import numpy as np
import random

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryActionResult
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import RobotState as RobotStateMoveIt
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Int32
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from niryo_robot_msgs.msg import RobotState, RPY

# Services
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from niryo_robot_msgs.srv import Trigger
from niryo_robot_commander.srv import GetFK, GetIK
from niryo_robot_msgs.srv import SetInt

# Enums
from niryo_robot_commander.command_enums import MoveCommandType, ArmCommanderException


class ArmCommander:
    """
    Object which command the Arm via MoveIt
    """

    def __init__(self, parameters_validator):

        # Getting reference frame from robot_commander.launch
        self.__reference_frame = rospy.get_param("~reference_frame")

        # - Subscribers
        joint_controller_base_name = rospy.get_param("~joint_controller_name")
        rospy.Subscriber('{}/follow_joint_trajectory/goal'.format(joint_controller_base_name),
                         FollowJointTrajectoryActionGoal, self.__callback_new_goal)

        rospy.Subscriber('{}/follow_joint_trajectory/result'.format(joint_controller_base_name),
                         FollowJointTrajectoryActionResult, self.__callback_goal_result)

        rospy.Subscriber('{}/follow_joint_trajectory/feedback'.format(joint_controller_base_name),
                         FollowJointTrajectoryActionFeedback, self.__callback_current_feedback)

        self.__traj_goal_pub = rospy.Publisher('{}/follow_joint_trajectory/goal'.format(joint_controller_base_name),
                                               FollowJointTrajectoryActionGoal, queue_size=1)

        self.__joints = None
        self.__joints_name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        rospy.Subscriber('/joint_states', JointState, self.__callback_sub_joint_states)

        # - Direct topic to joint_trajectory_controller
        self.__current_goal_id = None
        self.__current_feedback = None
        self.__current_goal_result = GoalStatus.LOST

        self.__joint_trajectory_publisher = rospy.Publisher('{}/command'.format(joint_controller_base_name),
                                                            JointTrajectory, queue_size=10)
        self.__reset_controller_service = rospy.ServiceProxy('/niryo_robot/joints_interface/steppers_reset_controller',
                                                             Trigger)

        # -- Commanders
        # - Move It Commander
        # Get Arm MoveGroupCommander
        self.__arm = moveit_commander.MoveGroupCommander(rospy.get_param("~move_group_commander_name"))
        # Get end effector link
        self.__end_effector_link = self.__arm.get_end_effector_link()

        # Set pose reference frame
        self.__arm.set_pose_reference_frame(self.__reference_frame)

        # Set planning parameters
        self.__arm.allow_replanning(rospy.get_param("~allow_replanning"))
        self.__arm.set_goal_joint_tolerance(rospy.get_param("~goal_joint_tolerance"))
        self.__arm.set_goal_position_tolerance(rospy.get_param("~goal_position_tolerance"))
        self.__arm.set_goal_orientation_tolerance(rospy.get_param("~goal_orientation_tolerance"))

        rospy.loginfo("Arm commander - MoveIt! successfully connected to move_group '{}'".format(self.__arm.get_name()))
        rospy.logdebug("Arm commander - MoveIt! will move '{}' in the"
                       " planning_frame '{}'".format(self.__end_effector_link, self.__arm.get_planning_frame()))

        # Validation
        self.__parameters_validator = parameters_validator

        # Event which allows to timeout if trajectory take too long
        self.__traj_finished_event = threading.Event()

        # Others params
        self.__trajectory_minimum_timeout = rospy.get_param("~trajectory_minimum_timeout")
        self.__compute_plan_max_tries = rospy.get_param("~compute_plan_max_tries")

        # - CALLABLE SERVICES

        # Arm velocity
        self.__max_velocity_scaling_factor = 100  # Start robot with max velocity
        self.__max_velocity_scaling_factor_pub = rospy.Publisher(
            '/niryo_robot/max_velocity_scaling_factor', Int32, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1), self.__publish_arm_max_velocity_scaling_factor)
        rospy.Service('/niryo_robot_commander/set_max_velocity_scaling_factor', SetInt,
                      self.__callback_set_max_velocity_scaling_factor)

        # Kinematics
        rospy.Service('/niryo_robot/kinematics/forward', GetFK,
                      self.__callback_get_forward_kinematics)
        rospy.Service('/niryo_robot/kinematics/inverse', GetIK,
                      self.__callback_get_inverse_kinematics)

    # -- Publishers call
    def __publish_arm_max_velocity_scaling_factor(self, _):
        """
        Publish Integer between 1 and 100 which correspond to the function name :)
        :param _: TimeEvent object which is not used
        :return: None
        """
        msg = Int32()
        msg.data = self.__max_velocity_scaling_factor
        self.__max_velocity_scaling_factor_pub.publish(msg)

    def __set_position_hold_mode(self):
        """
        Stop the Robot where it is
        :return: None
        """
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.points = []
        self.__joint_trajectory_publisher.publish(msg)

    def __reset_controller(self):
        self.__reset_controller_service()
        # very important delay to avoid unexpected issues from ros_control
        rospy.sleep(0.05)

    # -- Callbacks
    def __callback_sub_joint_states(self, joint_states):
        self.__joints = list(joint_states.position[:6])

    def __callback_new_goal(self, msg):
        """
        Happens when a new goal is published. Set the current goal
        :param msg: Object containing info on the new goal
        :type msg: FollowJointTrajectoryActionGoal
        :return: None
        """
        # rospy.loginfo('CALLBACK NEW GOAL')
        self.__current_goal_id = msg.goal_id.id
        rospy.logdebug("Arm commander - Got a goal id : {}".format(self.__current_goal_id))

    def __callback_current_feedback(self, msg):
        self.__current_feedback = msg

    def __callback_goal_result(self, msg):
        """
        Function called when the action is finished
        :param msg:
        :return:
        """
        if msg.status.goal_id.id == self.__current_goal_id:
            self.__current_goal_result = msg.status.status
            rospy.logdebug("Arm commander - Result : ".format(self.__current_goal_result))
            self.__traj_finished_event.set()
        else:
            rospy.loginfo("Arm commander - Received result, WRONG GOAL ID")

    def __callback_set_max_velocity_scaling_factor(self, req):
        if not 0 < req.value <= 100:
            return {'status': CommandStatus.INVALID_PARAMETERS, 'message': 'Value must be between 1 and 100'}
        try:
            self.__set_max_velocity_scaling_factor(req.value / 100.0)
        except ArmCommanderException as e:
            return {'status': CommandStatus.ARM_COMMANDER_FAILURE, 'message': e.message}
        self.__max_velocity_scaling_factor = req.value
        self.__publish_arm_max_velocity_scaling_factor(None)
        return {'status': CommandStatus.SUCCESS, 'message': 'Success'}

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

    # -- Executors
    def __compute_and_execute_plan_to_target(self):
        """
        Firstly try to compute the plan to the set target.
        If fails a first time, will try ComputePMaxTries times to stop the arm and recompute the plan
        Then, execute the plan
        :return: status, message
        """
        for tries in range(self.__compute_plan_max_tries + 1):  # We are going to try 3 times
            # if we are re-trying, first stop current plan
            if tries > 0:
                self.stop_current_plan()
                rospy.sleep(0.1)

            plan = self.__get_computed_plan()
            if not plan:
                raise ArmCommanderException(
                    CommandStatus.PLAN_FAILED, "MoveIt failed to compute the plan.")

            self.__reset_controller()
            rospy.logdebug("Arm commander - Send MoveIt trajectory to controller.")
            status, message = self.__execute_plan(plan)

            if status != CommandStatus.SHOULD_RESTART:
                return status, message
            if tries >= self.__compute_plan_max_tries:
                rospy.logerr("Arm commander - Big failure from the controller. Try to restart the robot")
                return CommandStatus.SHOULD_RESTART, "Please restart the robot and try again."
            rospy.logwarn("Arm commander - Will retry to compute "
                          "& execute trajectory {} time(s)".format(self.__compute_plan_max_tries - tries))

    def __compute_and_execute_trajectory(self, list_poses, dist_smoothing=0.01):
        """
        Hard try to to smooth Trajectories, not really working at the moment
        :param list_poses: list of Pose object
        :param dist_smoothing: smoothing distance
        :return: None
        """
        # rospy.logerr(list_poses)
        list_plans = []
        new_state = RobotStateMoveIt()
        new_state.joint_state.header.stamp = rospy.Time.now()
        new_state.joint_state.name = self.__joints_name
        # rospy.logerr(list_poses)
        for i, pose_target in enumerate(list_poses):
            # Getting plan to target
            self.__arm.set_pose_target(pose_target, self.__end_effector_link)
            list_plans.append(self.__arm.plan())

            # Generating new start state for future iteration
            success, new_state.joint_state.position = self.get_inverse_kinematics(pose=pose_target)
            if not success:
                self.__arm.set_start_state_to_current_state()
                raise ArmCommanderException(CommandStatus.INVERT_KINEMATICS_FAILURE, "IK Fail")
            self.__arm.set_start_state(new_state)

        self.__arm.set_start_state_to_current_state()
        plan = self.__link_plans(dist_smoothing, *list_plans)
        # self.__display_traj(plan, id_=int(1000 * dist_smoothing))
        return self.__execute_plan(plan)

    def __display_traj(self, plan, id_=1):
        topic_display = 'visualization_marker_array'
        if topic_display in rospy.get_published_topics():
            markers_array = rospy.wait_for_message(topic_display, MarkerArray).markers
        else:
            markers_array = []
        marker_pub = rospy.Publisher(topic_display, MarkerArray,
                                     queue_size=10, latch=True)

        cardboard_marker = Marker()
        cardboard_marker.header.frame_id = "world"
        cardboard_marker.header.stamp = rospy.Time.now()
        cardboard_marker.ns = "trajectory"
        cardboard_marker.id = id_
        cardboard_marker.type = cardboard_marker.LINE_STRIP
        cardboard_marker.action = cardboard_marker.ADD

        cardboard_marker.scale.x = 0.01

        cardboard_marker.color.r = random.random()
        cardboard_marker.color.g = random.random()
        cardboard_marker.color.b = random.random()
        cardboard_marker.color.a = 0.8

        cardboard_marker.points = [self.get_forward_kinematics(joints.positions).position for joints in
                                   plan.joint_trajectory.points]

        cardboard_marker.lifetime = rospy.Duration(0)
        markers_array.append(cardboard_marker)
        marker_pub.publish(markers_array)

    def __get_computed_plan(self):
        """
        Get computed plan from MoveIt
        :return: the computed plan if MoveIt succeed else None
        """
        plan = self.__arm.plan()
        return None if not plan.joint_trajectory.points else plan

    def __compute_and_execute_cartesian_plan(self, list_poses):
        """
        Compute a cartesian plan according to list_poses and then, execute it
        The robot will follow a straight line between each points
        If the plan is not validate, raise ArmCommanderException
        :param list_poses:
        :return: status, message
        """
        try:
            plan = self.__compute_cartesian_plan(list_poses)
        except Exception:
            raise ArmCommanderException(CommandStatus.ARM_COMMANDER_FAILURE, "IK Fail")

        if plan is not None:
            return self.__execute_plan(plan)
        elif plan is None:  #
            raise ArmCommanderException(CommandStatus.NO_PLAN_AVAILABLE,
                                        "The goal cannot be reached with a linear trajectory")

    def __compute_cartesian_plan(self, list_poses):
        """
        Compute cartesian plan from a list of poses
        As 2 poses cannot be the same, the first operation is to filter list_poses to be sure this condition is met
        :param list_poses: list of Pose Object
        :return: Computed plan : RobotTrajectory object
        """
        if len(list_poses) == 0:
            return GoalStatus.REJECTED, "No Waypoints"
        for i in reversed(range(len(list_poses) - 1)):
            if self.poses_too_close(list_poses[i + 1], list_poses[i]):
                list_poses.pop(i)

        trajectory_plan, fraction = self.__arm.compute_cartesian_path(list_poses, eef_step=0.05, jump_threshold=0.0)

        # Check the fraction value : if 1.0, the trajectory can be linear;
        # else, the trajectory followed won't be linear.
        if fraction == 1.0:
            # delete the very first joints position which is the starting position (current), 
            # to avoid an error related to increasing time 
            del trajectory_plan.joint_trajectory.points[0]
            return trajectory_plan
        elif fraction < 1.0:
            return None

    def __execute_plan(self, plan):
        """
        Execute the plan given
        Firstly, calculate a timeout which is : 1.5 times *the estimated time of the plan*
        Then send execute command to MoveIt and wait until the execution finished or the timeout happens
        :param plan: Computed plan
        :type plan: RobotTrajectory
        :return: CommandStatus, message
        """
        if not plan:
            raise ArmCommanderException(CommandStatus.NO_PLAN_AVAILABLE,
                                        "You are trying to execute a plan which doesn't exist")
        # Reset
        self.__traj_finished_event.clear()
        self.__current_goal_id = None
        self.__current_goal_result = GoalStatus.LOST

        trajectory_time_out = max(1.5 * self.__get_plan_time(plan), self.__trajectory_minimum_timeout)

        # Send trajectory and wait
        self.__arm.execute(plan, wait=False)
        if self.__traj_finished_event.wait(trajectory_time_out):
            if self.__current_goal_result == GoalStatus.SUCCEEDED:
                return CommandStatus.SUCCESS, "Command has been successfully processed"
            elif self.__current_goal_result == GoalStatus.PREEMPTED:
                return CommandStatus.STOPPED, "Command has been successfully stopped"
            elif self.__current_goal_result == GoalStatus.ABORTED:
                # if joint_trajectory_controller aborts the goal, it will still try to
                # finish executing the trajectory --> so we ask it to stop from here
                # http://wiki.ros.org/joint_trajectory_controller -> preemption policy
                # Send an empty trajectory from the topic interface
                self.__set_position_hold_mode()
                abort_str = "Command has been aborted due to a collision or " \
                            "a motor not able to follow the given trajectory"
                return CommandStatus.CONTROLLER_PROBLEMS, abort_str

            else:  # problem from ros_control itself
                self.__current_goal_id = None
                return CommandStatus.SHOULD_RESTART, ""
        else:
            # This timeout will happen if something fails in ros_control
            # It is not related to a trajectory failure
            self.__current_goal_id = None
            return CommandStatus.SHOULD_RESTART, ""

    # --- Callable functions

    def stop_arm(self):
        rospy.loginfo("Arm commander - Send STOP to arm")
        self.__arm.stop()

    def stop_current_plan(self):
        rospy.loginfo("Arm commander - Send STOP to arm and RESET to controller")
        self.__arm.stop()
        self.__reset_controller()

    def set_joint_target(self, arm_cmd):
        """
        Set controller target to a joint target
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        joints = list(arm_cmd.joints)
        self.__validate_params_move(MoveCommandType.JOINTS, joints)
        return self.__set_joint_target_moveit(joints)

    def __set_joint_target_moveit(self, joints):
        """
        Set MoveIt target to a joint target
        Then execute the trajectory to the target
        :param joints: joints list
        :type : list[float]
        :return: status, message
        """
        self.__arm.set_joint_value_target(joints)
        return self.__compute_and_execute_plan_to_target()

    def set_pose_target_from_cmd(self, arm_cmd):
        """
        Set MoveIt target to a pose target with RPY
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        self.__validate_params_move(MoveCommandType.POSE, arm_cmd.position, arm_cmd.rpy)
        x, y, z = arm_cmd.position.x, arm_cmd.position.y, arm_cmd.position.z
        roll, pitch, yaw = arm_cmd.rpy.roll, arm_cmd.rpy.pitch, arm_cmd.rpy.yaw
        return self.__set_pose_target_moveit(x, y, z, roll, pitch, yaw)

    def __set_pose_target_moveit(self, x, y, z, roll, pitch, yaw):
        self.__arm.set_pose_target([x, y, z, roll, pitch, yaw], self.__end_effector_link)
        return self.__compute_and_execute_plan_to_target()

    def set_position_target(self, arm_cmd):
        """
        Set MoveIt target to a position target
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        self.__validate_params_move(MoveCommandType.POSITION, arm_cmd.position)
        x, y, z = arm_cmd.position.x, arm_cmd.position.y, arm_cmd.position.z
        self.__arm.set_position_target([x, y, z], self.__end_effector_link)
        return self.__compute_and_execute_plan_to_target()

    def set_rpy_target(self, arm_cmd):
        """
        Set MoveIt target to a rpy target
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        self.__validate_params_move(MoveCommandType.RPY, arm_cmd.rpy)
        roll, pitch, yaw = arm_cmd.rpy.roll, arm_cmd.rpy.pitch, arm_cmd.rpy.yaw
        self.__arm.set_rpy_target([roll, pitch, yaw], self.__end_effector_link)
        return self.__compute_and_execute_plan_to_target()

    def set_pose_quat_target(self, arm_cmd):
        """
        Set MoveIt target to a Pose target with quaternion
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        self.__validate_params_move(MoveCommandType.POSE_QUAT, arm_cmd.position, arm_cmd.orientation)
        x, y, z = arm_cmd.position.x, arm_cmd.position.y, arm_cmd.position.z
        q_x, q_y, q_z, q_w = arm_cmd.orientation.x, arm_cmd.orientation.y, arm_cmd.orientation.z, arm_cmd.orientation.w
        self.__arm.set_pose_target([x, y, z, q_x, q_y, q_z, q_w], self.__end_effector_link)
        return self.__compute_and_execute_plan_to_target()

    def set_pose_quat_from_pose(self, pose):
        """
        Set MoveIt target to a Pose target with quaternion
        Then execute the trajectory to the target
        :param pose: Pose message containing target values
        :type : Pose
        :return: status, message
        """
        self.__validate_params_move(MoveCommandType.POSE_QUAT, pose.position, pose.orientation)
        self.__arm.set_pose_target(pose, self.__end_effector_link)
        return self.__compute_and_execute_plan_to_target()

    def set_shift_pose_target(self, arm_cmd):
        """
        Set MoveIt target to a shifted target from actual position
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        self.__validate_params_move(MoveCommandType.SHIFT_POSE, arm_cmd.shift)
        self.__arm.shift_pose_target(arm_cmd.shift.axis_number, arm_cmd.shift.value, self.__end_effector_link)
        return self.__compute_and_execute_plan_to_target()

    def set_shift_linear_pose_target(self, arm_cmd):
        """
        Set MoveIt target to a shifted target from actual position
        Then execute the cartesian trajectory to the target 
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        self.__validate_params_move(MoveCommandType.SHIFT_LINEAR_POSE, arm_cmd.shift)

        axis_number = arm_cmd.shift.axis_number
        shift_value = arm_cmd.shift.value

        # Get current end effector pose
        actual_pose = self.__arm.get_current_pose().pose

        # Get list [x, y, z, roll, pitch, yaw] from pose stamped
        pose_list = self.pose_to_list(actual_pose)

        # Apply shift on pose 
        pose_list[axis_number] += shift_value

        # Get pose stamped from target pose 
        msg_pose = self.list_to_pose(pose_list)

        # Check if command is really close to the current position
        if self.poses_too_close(msg_pose, actual_pose):
            return CommandStatus.SUCCESS, "Command was already satisfied"

        # set arm pose target
        self.__arm.set_pose_target(pose_list, self.__end_effector_link)

        return self.__compute_and_execute_cartesian_plan([msg_pose])

    def set_linear_trajectory(self, arm_cmd):
        """
        Set MoveIt target to a Pose target with RPY
        Then execute a Linear trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        x, y, z = arm_cmd.position.x, arm_cmd.position.y, arm_cmd.position.z
        roll, pitch, yaw = arm_cmd.rpy.roll, arm_cmd.rpy.pitch, arm_cmd.rpy.yaw
        x_q, y_q, z_q, w_q = quaternion_from_euler(roll, pitch, yaw)
        msg_pose = Pose(Point(x, y, z), Quaternion(x_q, y_q, z_q, w_q))

        current_pose = self.__arm.get_current_pose().pose

        # If the goal is really close to the current pose, 
        # avoid useless calculations and return immediately
        if self.poses_too_close(msg_pose, current_pose):
            return CommandStatus.SUCCESS, "Command was already satisfied"

        else:
            return self.__compute_and_execute_cartesian_plan([msg_pose])

    # - Trajectory
    def execute_trajectory(self, arm_cmd):
        """
        Version 1 : Go to first pose using "classic" trajectory then compute cartesian path between all points
        Version 2 : Going to all poses one by one using "classical way"
        Version 3 : Generate all plans, merge them & smooth transitions

        """
        list_poses = arm_cmd.list_poses
        if len(list_poses) == 0:
            return CommandStatus.NO_PLAN_AVAILABLE, "Can't generate plan from a list of length 0"

        dist_smoothing = arm_cmd.dist_smoothing

        if dist_smoothing == 0.0:
            if len(list_poses) < 3:  # Classical moves
                for pose in list_poses:
                    ret = self.set_pose_quat_from_pose(pose)
                    if ret[0] != CommandStatus.SUCCESS:
                        return ret
                return CommandStatus.SUCCESS, "Trajectory is Good!"
            else:  # Linear path
                # We are going to the initial pose using "classical" method
                pose_init = list_poses.pop(0)
                self.set_pose_quat_from_pose(pose_init)
                return self.__compute_and_execute_cartesian_plan(list_poses)

        else:
            return self.__compute_and_execute_trajectory(list_poses, dist_smoothing=dist_smoothing)

    def draw_spiral_trajectory(self, arm_cmd):
        """
        Generate waypoints to draw a spiral and generate a cartesian path with these waypoints
        Then execute the trajectory plan

        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        min_radius = 0.003
        radius, angle_step, total_steps = [float(arg) for arg in arm_cmd.args]

        center_position = self.__arm.get_current_pose().pose.position
        target_pose = self.__arm.get_current_pose().pose

        waypoints = []
        angle_step_rad = math.radians(angle_step)
        angle = 0
        for i in range(2):
            for dist_from_center in np.linspace(0, float(radius), num=total_steps):
                if i == 1:
                    dist_from_center = radius - dist_from_center
                dist_from_center += min_radius
                y_offset = dist_from_center * math.cos(angle)
                z_offset = dist_from_center * math.sin(angle)
                target_pose.position.y = center_position.y + y_offset
                target_pose.position.z = center_position.z + z_offset
                waypoints.append(copy.deepcopy(target_pose))
                angle += angle_step_rad
        return self.__compute_and_execute_cartesian_plan(waypoints)

    def __retime_plan(self, plan, velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0, optimize=False):
        """
        Take a plan and retime it
        """
        start_state = self.__get_plan_start_robot_state(plan)

        if optimize:
            algorithm = "time_optimal_trajectory_generation"
        else:
            algorithm = "iterative_time_parameterization"

        plan_out = self.__arm.retime_trajectory(start_state, plan,
                                                velocity_scaling_factor=velocity_scaling_factor,
                                                acceleration_scaling_factor=acceleration_scaling_factor,
                                                algorithm=algorithm
                                                )
        return plan_out

    def __get_plan_start_robot_state(self, plan):
        start_state = RobotStateMoveIt()
        start_state.joint_state.header.frame_id = "world"
        start_state.joint_state.position = plan.joint_trajectory.points[0].positions
        start_state.joint_state.name = self.__joints_name
        return start_state

    def __link_plans(self, dist_smoothing=0.0, *plans):
        """
        Link plans together with a smooth transition between each of them
        """

        def find_limit_point(plan_, dist_smooth, reverse):
            len_plan = len(plan_.joint_trajectory.points)
            ind_ref = -1 if reverse else 0
            reference_pos = self.get_forward_kinematics(plan_.joint_trajectory.points[ind_ref].positions)
            for raw_ind in range(1, len_plan):
                if reverse:
                    ind = len_plan - raw_ind - 1
                else:
                    ind = raw_ind
                pose_i = self.get_forward_kinematics(plan_.joint_trajectory.points[ind].positions)
                dist_pose_to_target = self.dist_2_poses(reference_pos, pose_i)
                if dist_pose_to_target > dist_smooth:
                    return ind
            else:
                if reverse:
                    return 0
                else:
                    return len_plan - 1

        smooth_zones = []
        offset = 0
        for plan_in, plan_out in zip(plans[:-1], plans[1:]):
            begin_index = offset + find_limit_point(plan_in, dist_smoothing, reverse=True)
            offset += len(plan_in.joint_trajectory.points) - 1
            mid_index = offset
            end_index = mid_index + find_limit_point(plan_out, dist_smoothing, reverse=False)
            smooth_zones.append((begin_index, mid_index, end_index))

        big_plan = plans[0]
        time_offset = big_plan.joint_trajectory.points[-1].time_from_start

        for plan in plans[1:]:
            for i in range(1, len(plan.joint_trajectory.points)):
                plan.joint_trajectory.points[i].time_from_start += time_offset

            big_plan.joint_trajectory.points += plan.joint_trajectory.points[1:]

            time_offset += plan.joint_trajectory.points[-1].time_from_start

        def ind_to_pos(ind):
            return np.array(big_plan.joint_trajectory.points[ind].positions)

        def bezier(p0, p1, p2, t):
            return ((1 - t) ** 2) * p0 + 2 * (1 - t) * t * p1 + (t ** 2) * p2

        for smooth_zone in smooth_zones:
            begin_index, mid_index, end_index = smooth_zone
            begin_positions, mid_positions, end_positions = map(ind_to_pos, [begin_index, mid_index, end_index])
            for point_index in range(begin_index + 1, end_index):
                ratio = float(point_index - begin_index) / (end_index - begin_index)
                bezier_result = bezier(begin_positions, mid_positions, end_positions, ratio)
                big_plan.joint_trajectory.points[point_index].positions = bezier_result

        final_plan = self.__retime_plan(big_plan, optimize=True)
        return final_plan

    def __set_max_velocity_scaling_factor(self, percentage):
        """
        Ask MoveIt to set the relative speed to (percentage)%
        :param percentage:
        :return: None
        """
        self.__arm.set_max_velocity_scaling_factor(percentage)

    # - General Purposes

    def __validate_params_move(self, command_type, *args):
        """
        Validate parameters according to command_type
        The function will raise an error if command is not valid
        :param command_type:
        :type command_type: MoveCommandType
        :param args: parameters
        :return: None
        """
        if command_type == MoveCommandType.JOINTS:
            self.__parameters_validator.validate_joints(args[0])
        elif command_type == MoveCommandType.POSE:
            self.__parameters_validator.validate_position(args[0])
            self.__parameters_validator.validate_orientation(args[1])
        elif command_type == MoveCommandType.POSITION:
            self.__parameters_validator.validate_position(args[0])
        elif command_type == MoveCommandType.RPY:
            self.__parameters_validator.validate_orientation(args[0])
        elif command_type == MoveCommandType.EXECUTE_TRAJ:
            self.__parameters_validator.validate_trajectory(args[0])
        elif command_type == MoveCommandType.POSE_QUAT:
            self.__parameters_validator.validate_position(args[0])
            self.__parameters_validator.validate_orientation_quaternion(args[1])
        elif command_type == MoveCommandType.SHIFT_POSE:
            self.__parameters_validator.validate_shift_pose(args[0])
        elif command_type == MoveCommandType.SHIFT_LINEAR_POSE:
            self.__parameters_validator.validate_shift_pose(args[0])
        else:
            raise ArmCommanderException(CommandStatus.UNKNOWN_COMMAND, "Wrong command type")

    @staticmethod
    def __get_plan_time(plan):
        """
        Extract duration from a plan. If it cannot be done, raise ArmCommanderException
        :param plan: plan given by MoveIt
        :type plan: RobotTrajectory
        :return: duration in seconds
        """
        if plan and plan.joint_trajectory.points:
            return plan.joint_trajectory.points[-1].time_from_start.to_sec()
        else:
            raise ArmCommanderException(CommandStatus.NO_PLAN_AVAILABLE,
                                        "No current plan found")

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

        quaternion = [response.pose_stamped[1].pose.orientation.x, response.pose_stamped[1].pose.orientation.y,
                      response.pose_stamped[1].pose.orientation.z, response.pose_stamped[1].pose.orientation.w]

        rpy = euler_from_quaternion(quaternion)
        quaternion = (round(quaternion[0], 3), round(quaternion[1], 3), round(quaternion[2], 3),
                      round(quaternion[3], 3))
        point = (round(response.pose_stamped[1].pose.position.x, 3),
                 round(response.pose_stamped[1].pose.position.y, 3),
                 round(response.pose_stamped[1].pose.position.z, 3))

        return RobotState(Point(*point), RPY(*rpy), Quaternion(*quaternion))

    def get_inverse_kinematics(self, pose):
        try:
            rospy.wait_for_service('compute_ik', 2)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Arm commander - Impossible to connect to IK service : " + str(e))
            return False, []
        try:
            moveit_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
            req = PositionIKRequest()
            req.group_name = self.__arm.get_name()
            req.ik_link_name = self.__end_effector_link

            req.robot_state.joint_state.name = self.__joints_name
            req.robot_state.joint_state.position = self.__joints
            req.pose_stamped.pose = pose

            response = moveit_ik(req)
        except rospy.ServiceException as e:
            rospy.logerr("Arm commander - Service call failed: {}".format(e))
            return False, []
        if response.error_code.val == MoveItErrorCodes.NO_IK_SOLUTION:
            rospy.logerr("Arm commander - MoveIt didn't find an IK solution")
            return False, []
        elif response.error_code.val != MoveItErrorCodes.SUCCESS:
            rospy.logerr("Arm commander - IK Failed : code error {}".format(response.error_code.val))
            return False, []
        return True, list(response.solution.joint_state.position[:6])

    # - Useful functions

    @staticmethod
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

    @staticmethod
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

    @staticmethod
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

    @staticmethod
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
