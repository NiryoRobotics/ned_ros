#!/usr/bin/env python

# Lib
import rospy
from tf.transformations import quaternion_from_euler

import copy
import math
import numpy as np

from .trajectories_executor import TrajectoriesExecutor
from .jog_controller import JogController
from .trajectory_handler import TrajectoryHandlerNode
from .utils import (
    list_to_pose,
    pose_to_list,
    dist_2_poses,
    dist_2_points,
    poses_too_close,
    angle_between_2_points,
)

# Command Status
from niryo_robot_msgs.msg import CommandStatus, RobotState

# Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import RobotState as RobotStateMoveIt
from niryo_robot_arm_commander.msg import ArmMoveCommand
from niryo_robot_msgs.msg import RPY

# Enums
from .command_enums import ArmCommanderException


class ArmCommander:
    """
    Object which set the arm goals to MoveIt
    """

    def __init__(self, arm_state):
        rospy.logdebug("Arm Commander - Entering in Init")
        self.__arm_state = arm_state
        self.__joints_name = self.__arm_state.joints_name

        # -- Commanders
        # - Move It Commander
        self.__arm = self.__arm_state.arm
        self.__end_effector_link = self.__arm.get_end_effector_link()

        # Executor
        self.__traj_executor = TrajectoriesExecutor(self.__arm)
        self.__traj_manager = TrajectoryHandlerNode(self.__arm_state, self.__traj_executor)

        # Validation
        self.__parameters_validator = self.__arm_state.parameters_validator

        # - Frames managers
        self.__transform_handler = self.__arm_state.transform_handler
        self.__kinematics_handler = self.__arm_state.kinematics

        # Jog Controller
        self.__jog_controller = JogController(self.__arm_state)

    @property
    def trajectories_executor(self):
        return self.__traj_executor

    # - PTP Trajectory
    def set_joint_target(self, arm_cmd):
        """
        Set controller target to a joint target
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        joints = list(arm_cmd.joints)
        self.__validate_params_move(ArmMoveCommand.JOINTS, joints)
        self.__check_collision_in_joints_target(joints)

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
        return self.__traj_executor.compute_and_execute_plan_to_target()

    def set_pose_target_from_cmd(self, arm_cmd):
        """
        Set MoveIt target to a pose target with RPY
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        x, y, z = arm_cmd.position.x, arm_cmd.position.y, arm_cmd.position.z
        roll, pitch, yaw = arm_cmd.rpy.roll, arm_cmd.rpy.pitch, arm_cmd.rpy.yaw
        return self.__set_pose_target_moveit(x, y, z, roll, pitch, yaw)

    def __set_pose_target_moveit(self, x, y, z, roll, pitch, yaw):
        tcp_pose = list_to_pose([x, y, z, roll, pitch, yaw])
        self.__transform_handler.display_target_pose(tcp_pose)

        ee_pose = self.__transform_handler.tcp_to_ee_link_pose_target(tcp_pose, self.__end_effector_link)
        self.__validate_params_move(ArmMoveCommand.POSE_QUAT, ee_pose.position, ee_pose.orientation)

        self.__arm.set_pose_target(ee_pose, self.__end_effector_link)

        return self.__traj_executor.compute_and_execute_plan_to_target()

    def set_position_target(self, arm_cmd):
        """
        Set MoveIt target to a position target
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        ee_position = self.__transform_handler.tcp_to_ee_link_position_target(arm_cmd.position,
                                                                              self.__end_effector_link)
        self.__validate_params_move(ArmMoveCommand.POSITION, ee_position)

        x, y, z = ee_position.x, ee_position.y, ee_position.z
        self.__arm.set_position_target([x, y, z], self.__end_effector_link)
        return self.__traj_executor.compute_and_execute_plan_to_target()

    def set_rpy_target(self, arm_cmd):
        """
        Set MoveIt target to a rpy target
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        tcp_roll, tcp_pitch, tcp_yaw = (
            arm_cmd.rpy.roll,
            arm_cmd.rpy.pitch,
            arm_cmd.rpy.yaw,
        )
        ee_roll, ee_pitch, ee_yaw = self.__transform_handler.tcp_to_ee_link_rpy_target(
            tcp_roll, tcp_pitch, tcp_yaw, self.__end_effector_link
        )
        self.__validate_params_move(ArmMoveCommand.RPY, RPY(ee_roll, ee_pitch, ee_yaw))
        self.__arm.set_rpy_target([ee_roll, ee_pitch, ee_yaw], self.__end_effector_link)
        return self.__traj_executor.compute_and_execute_plan_to_target()

    def set_pose_quat_target(self, arm_cmd):
        """
        Set MoveIt target to a Pose target with quaternion
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        tcp_pose = Pose(arm_cmd.position, arm_cmd.orientation)
        self.__transform_handler.display_target_pose(tcp_pose)
        ee_pose = self.__transform_handler.tcp_to_ee_link_pose_target(tcp_pose, self.__end_effector_link)
        self.__validate_params_move(ArmMoveCommand.POSE_QUAT, ee_pose.position, ee_pose.orientation)
        self.__arm.set_pose_target(ee_pose, self.__end_effector_link)
        return self.__traj_executor.compute_and_execute_plan_to_target()

    def set_pose_quat_from_pose(self, pose):
        """
        Set MoveIt target to a Pose target with quaternion
        Then execute the trajectory to the target
        :param pose: Pose message containing target values
        :type : Pose
        :return: status, message
        """
        self.__validate_params_move(ArmMoveCommand.POSE_QUAT, pose.position, pose.orientation)
        ee_pose = self.__transform_handler.tcp_to_ee_link_pose_target(pose, self.__end_effector_link)
        self.__arm.set_pose_target(ee_pose, self.__end_effector_link)
        return self.__traj_executor.compute_and_execute_plan_to_target()

    # - Shift Trajectory
    def set_shift_pose_target(self, arm_cmd):
        """
        Set MoveIt target to a shifted target from actual position
        Then execute the trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        self.__validate_params_move(ArmMoveCommand.SHIFT_POSE, arm_cmd.shift)
        self.__arm.shift_pose_target(arm_cmd.shift.axis_number, arm_cmd.shift.value, self.__end_effector_link)
        return self.__traj_executor.compute_and_execute_plan_to_target()

    def set_shift_linear_pose_target(self, arm_cmd):
        """
        Set MoveIt target to a shifted target from actual position
        Then execute the cartesian trajectory to the target
        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        self.__validate_params_move(ArmMoveCommand.SHIFT_LINEAR_POSE, arm_cmd.shift)

        axis_number = arm_cmd.shift.axis_number
        shift_value = arm_cmd.shift.value

        # Get current end effector pose
        actual_pose = self.__arm.get_current_pose().pose

        # Get list [x, y, z, roll, pitch, yaw] from pose stamped
        pose_list = pose_to_list(actual_pose)

        # Apply shift on pose
        pose_list[axis_number] += shift_value

        # Get pose stamped from target pose
        msg_pose = list_to_pose(pose_list)

        # Check if command is really close to the current position
        if poses_too_close(msg_pose, actual_pose):
            return CommandStatus.SUCCESS, "Command was already satisfied"

        # set arm pose target
        self.__arm.set_pose_target(pose_list, self.__end_effector_link)
        return self.__traj_executor.compute_and_execute_cartesian_plan(
            [msg_pose],
            self.__arm_state.velocity_scaling_factor,
            self.__arm_state.acceleration_scaling_factor,
        )

    # - Linear Trajectory
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
        tcp_pose = Pose(Point(x, y, z), Quaternion(x_q, y_q, z_q, w_q))
        self.__transform_handler.display_target_pose(tcp_pose)
        ee_pose = self.__transform_handler.tcp_to_ee_link_pose_target(tcp_pose, self.__end_effector_link)

        current_pose = self.__arm.get_current_pose().pose

        # If the goal is really close to the current pose,
        # avoid useless calculations and return immediately
        if poses_too_close(ee_pose, current_pose):
            return CommandStatus.SUCCESS, "Command was already satisfied"

        else:
            return self.__traj_executor.compute_and_execute_cartesian_plan(
                [ee_pose],
                self.__arm_state.velocity_scaling_factor,
                self.__arm_state.acceleration_scaling_factor,
            )

    # - Spiral Trajectory
    def draw_spiral_trajectory(self, arm_cmd):
        """
        Generate waypoints to draw a spiral and generate a cartesian path with these waypoints
        Then execute the trajectory plan

        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        min_radius = 0.003
        radius, angle_step, total_steps, plan = [float(arg) for arg in arm_cmd.args]

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

                if plan == 1:
                    y_offset = dist_from_center * math.cos(angle)
                    z_offset = dist_from_center * math.sin(angle)
                    target_pose.position.y = center_position.y + y_offset
                    target_pose.position.z = center_position.z + z_offset
                elif plan == 2:
                    x_offset = dist_from_center * math.cos(angle)
                    z_offset = dist_from_center * math.sin(angle)
                    target_pose.position.x = center_position.x + x_offset
                    target_pose.position.z = center_position.z + z_offset
                elif plan == 3:
                    x_offset = dist_from_center * math.cos(angle)
                    y_offset = dist_from_center * math.sin(angle)
                    target_pose.position.x = center_position.x + x_offset
                    target_pose.position.y = center_position.y + y_offset
                else:
                    rospy.logwarn("Spiral planer: bad plan parameter, must be between 1 and 3: "
                                  "[1 = yz plan, 2 = xz plan, 3 = xy plan]")
                    raise ArmCommanderException

                waypoints.append(copy.deepcopy(target_pose))
                angle += angle_step_rad

        return self.__traj_executor.compute_and_execute_cartesian_plan(
            waypoints,
            self.__arm_state.velocity_scaling_factor,
            self.__arm_state.acceleration_scaling_factor,
        )

    def draw_circle_trajectory(self, arm_cmd):
        """
        Generate waypoints to draw a circle and generate a cartesian path with these waypoints
        Then execute the trajectory plan

        :param arm_cmd: ArmMoveCommand message containing target values
        :type : ArmMoveCommand
        :return: status, message
        """
        target_pose = self.__arm.get_current_pose().pose
        center_point = arm_cmd.position
        dist_from_center = dist_2_points(target_pose.position, center_point)

        tmp_point = Point(
            round(target_pose.position.x - center_point.x, 4),
            round(target_pose.position.y - center_point.y, 4),
            round(target_pose.position.z - center_point.z, 4),
        )

        angle_off = angle_between_2_points(Point(0, 1, 0), tmp_point)

        waypoints = []
        angle_steps = 3
        for angle in range(0, 360 + angle_steps, angle_steps):
            angle_rad = math.radians(-angle) + angle_off
            y_offset = dist_from_center * math.cos(angle_rad)
            z_offset = dist_from_center * math.sin(angle_rad)
            target_pose.position.y = center_point.y + y_offset
            target_pose.position.z = center_point.z + z_offset
            waypoints.append(copy.deepcopy(target_pose))
        return self.__traj_executor.compute_and_execute_cartesian_plan(waypoints)

    # - Waypointed Trajectory
    def execute_waypointed_trajectory(self, arm_cmd):
        self.__validate_params_move(ArmMoveCommand.EXECUTE_FULL_TRAJ, arm_cmd.trajectory)
        return self.__traj_executor.execute_joint_trajectory(arm_cmd.trajectory)

    def execute_raw_waypointed_trajectory(self, arm_cmd):
        # go to start pose # move_joints
        self.__arm.set_joint_value_target(arm_cmd.trajectory.joint_trajectory.points[0].positions)
        # plan = self.__traj_executor.plan(arm_cmd.trajectory)
        # return self.__traj_executor.execute_plan(plan)
        return self.__traj_executor.execute_plan(arm_cmd.trajectory)

    def compute_and_execute_waypointed_trajectory(self, arm_cmd):
        try:
            status, message, plan = self.compute_waypointed_trajectory(arm_cmd)
        except ArmCommanderException as e:
            return e.status, e.message

        if status != CommandStatus.SUCCESS:
            return status, message

        return self.__traj_executor.execute_plan(plan)

    def compute_waypointed_trajectory(self, arm_cmd):
        """
        Version 1 : Go to first pose using "classic" trajectory then compute cartesian path between all points
        Version 2 : Going to all poses one by one using "classical way"
        Version 3 : Generate all plans, merge them & smooth transitions
        """
        list_tcp_poses = arm_cmd.list_poses
        if len(list_tcp_poses) == 0:
            return (
                CommandStatus.NO_PLAN_AVAILABLE,
                "Can't generate plan from a list of length 0",
                None,
            )
        list_ee_poses = [
            self.__transform_handler.tcp_to_ee_link_pose_target(tcp_pose, self.__end_effector_link)
            for tcp_pose in list_tcp_poses
        ]

        dist_smoothing = arm_cmd.dist_smoothing

        if dist_smoothing == 0.0:
            # We are going to the initial pose using "classical" method
            try:
                plan = self.__traj_executor.compute_cartesian_plan(list_ee_poses)
                if plan is None:
                    raise ArmCommanderException(CommandStatus.NO_PLAN_AVAILABLE, "")
                plan = self.__traj_executor.retime_plan(
                    plan,
                    self.__arm_state.velocity_scaling_factor,
                    self.__arm_state.acceleration_scaling_factor,
                    optimize=False,
                )

            except ArmCommanderException as e:
                if e.status == CommandStatus.NO_PLAN_AVAILABLE:
                    rospy.loginfo("Cartesian path computation failed, let's try another way!")
                    plan = self.__compute_trajectory(list_ee_poses, dist_smoothing=dist_smoothing)
                else:
                    raise e
        else:
            plan = self.__compute_trajectory(list_ee_poses, dist_smoothing=dist_smoothing)

        return CommandStatus.SUCCESS, "Trajectory is Good!", plan

    def __compute_trajectory(self, list_poses, dist_smoothing=0.01):
        """
        Hard try to to smooth Trajectories, not really working at the moment
        :param list_poses: list of Pose object
        :param dist_smoothing: smoothing distance
        :return: None
        """
        self.__arm.set_start_state_to_current_state()

        list_plans = []
        new_state = RobotStateMoveIt()
        new_state.joint_state.header.stamp = rospy.Time.now()
        new_state.joint_state.name = self.__joints_name
        new_state.joint_state.position = self.__arm.get_current_joint_values()

        for i, pose_target in enumerate(list_poses):
            # Getting plan to target
            # Try a linear path first
            partial_plan = self.__traj_executor.compute_cartesian_plan([pose_target])
            if not partial_plan:  # Try a PTP path if linear path failed
                rospy.loginfo("Linear path not found between {} joints and {} pose".format(
                    new_state.joint_state.position, pose_to_list(pose_target)))
                self.__arm.set_pose_target(pose_target, self.__end_effector_link)
                try:
                    partial_plan = self.__traj_executor.plan()
                except ArmCommanderException as e:
                    self.__arm.set_start_state_to_current_state()
                    raise e

            list_plans.append(copy.deepcopy(partial_plan))

            # Generating new start state for future iteration
            new_state.joint_state.position = (list_plans[-1].joint_trajectory.points[-1].positions)
            self.__arm.set_start_state(new_state)

        self.__arm.set_start_state_to_current_state()
        if dist_smoothing > 0:
            plan = self.__link_plans_with_smoothing(dist_smoothing, *list_plans)
        else:
            plan = self.__traj_executor.link_plans(*list_plans)
        self.display_traj(plan, id_=int(1000 * dist_smoothing))
        return plan

    def __link_plans_with_smoothing(self, dist_smoothing=0.0, *plans):
        """
        Link plans together with a smooth transition between each of them
        """

        def find_limit_point(plan_, dist_smooth, reverse):
            len_plan = len(plan_.joint_trajectory.points)
            ind_ref = -1 if reverse else 0
            reference_pos = self.__kinematics_handler.get_forward_kinematics_v2(
                plan_.joint_trajectory.points[ind_ref].positions)
            if reference_pos is None:
                reference_pos = RobotState()
            for raw_ind in range(1, len_plan):
                if reverse:
                    ind = len_plan - raw_ind - 1
                else:
                    ind = raw_ind
                pose_i = self.__kinematics_handler.get_forward_kinematics_v2(
                    plan_.joint_trajectory.points[ind].positions)
                if pose_i is None:
                    pose_i = RobotState()
                dist_pose_to_target = dist_2_poses(reference_pos, pose_i)

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
            offset += len(plan_in.joint_trajectory.points)
            mid_index = offset
            end_index = mid_index + find_limit_point(plan_out, dist_smoothing, reverse=False)
            smooth_zones.append((begin_index, mid_index, end_index))

        big_plan = plans[0]
        for plan in plans[1:]:
            big_plan.joint_trajectory.points.extend(plan.joint_trajectory.points[:])

        def ind_to_pos(ind):
            return np.array(big_plan.joint_trajectory.points[ind].positions)

        def bezier(p0, p1, p2, t):
            return ((1 - t)**2) * p0 + 2 * (1 - t) * t * p1 + (t**2) * p2

        for smooth_zone in smooth_zones:
            begin_index, mid_index, end_index = smooth_zone
            begin_positions, mid_positions, end_positions = map(ind_to_pos, [begin_index, mid_index, end_index])
            for point_index in range(begin_index + 1, end_index):
                ratio = float(point_index - begin_index) / (end_index - begin_index)
                bezier_result = bezier(begin_positions, mid_positions, end_positions, ratio)
                big_plan.joint_trajectory.points[point_index].positions = bezier_result

        final_plan = self.__traj_executor.retime_plan(
            big_plan,
            optimize=True,
            velocity_scaling_factor=self.__arm_state.velocity_scaling_factor,
            acceleration_scaling_factor=self.__arm_state.acceleration_scaling_factor,
        )
        return final_plan

    # - General Purposes
    def display_traj(self, plan, id_=1):
        if rospy.get_param("~display_trajectories"):
            points = []
            for joints in plan.joint_trajectory.points:
                forward_kinematics = (self.__kinematics_handler.get_forward_kinematics_v2(joints.positions))
                if forward_kinematics is None:
                    forward_kinematics = RobotState()
                points.append(forward_kinematics.position)
            self.__traj_executor.display_traj(points, id_)

    def __validate_params_move(self, command_type, *args):
        """
        Validate parameters according to command_type
        The function will raise an error if command is not valid
        :param command_type:
        :type command_type: ArmMoveCommand
        :param args: parameters
        :return: None
        """
        if command_type == ArmMoveCommand.JOINTS:
            self.__parameters_validator.validate_joints(args[0])
        elif command_type == ArmMoveCommand.POSE:
            self.__parameters_validator.validate_position(args[0])
            self.__parameters_validator.validate_orientation(args[1])
        elif command_type == ArmMoveCommand.POSITION:
            self.__parameters_validator.validate_position(args[0])
        elif command_type == ArmMoveCommand.RPY:
            self.__parameters_validator.validate_orientation(args[0])
        elif command_type in [
                ArmMoveCommand.EXECUTE_TRAJ,
                ArmMoveCommand.EXECUTE_FULL_TRAJ,
        ]:
            self.__parameters_validator.validate_trajectory(args[0])
        elif command_type == ArmMoveCommand.POSE_QUAT:
            self.__parameters_validator.validate_position(args[0])
            self.__parameters_validator.validate_orientation_quaternion(args[1])
        elif command_type == ArmMoveCommand.SHIFT_POSE:
            self.__parameters_validator.validate_shift_pose(args[0])
        elif command_type == ArmMoveCommand.SHIFT_LINEAR_POSE:
            self.__parameters_validator.validate_shift_pose(args[0])
        else:
            raise ArmCommanderException(CommandStatus.UNKNOWN_COMMAND, "Wrong command type")

    def __check_collision_in_joints_target(self, joints):
        """
        Check if the joints target implies a self collision. Raise an exception is a collision is detected.
        :param joints: list of joints value
        """
        try:
            robot_state_target = RobotStateMoveIt()
            robot_state_target.joint_state.header.frame_id = "world"
            robot_state_target.joint_state.position = joints
            robot_state_target.joint_state.name = self.__joints_name
            group_name = self.__arm.get_name()
            response = self.__parameters_validator.check_state_validity(robot_state_target, group_name, None)
            if not response.valid:
                if len(response.contacts) > 0:
                    rospy.logwarn(
                        "Arm commander - Joints target unreachable because of collision between %s and %s",
                        response.contacts[0].contact_body_1,
                        response.contacts[0].contact_body_2,
                    )
                    raise ArmCommanderException(
                        CommandStatus.INVALID_PARAMETERS,
                        "Target joints would lead to a collision between links {} and {} ".format(
                            response.contacts[0].contact_body_1,
                            response.contacts[0].contact_body_2,
                        ),
                    )
                else:  # didn't succeed to get the contacts on the real robot
                    rospy.logwarn(
                        "Arm commander - Joints target unreachable because of collision between two parts of Ned")
                    raise ArmCommanderException(
                        CommandStatus.INVALID_PARAMETERS,
                        "Target joints would lead to a collision between two parts of Ned",
                    )

        except rospy.ServiceException as e:
            rospy.logwarn("Arm commander - Failed to check state validity : " + str(e))

    @property
    def jog_controller(self):
        return self.__jog_controller
