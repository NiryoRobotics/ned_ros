#!/usr/bin/env python

# Lib
import rospy

import threading
import random

from .utils import poses_too_close

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryActionResult
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from moveit_msgs.msg import RobotState as RobotStateMoveIt
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Quaternion

# Services
from niryo_robot_msgs.srv import Trigger
from niryo_robot_msgs.srv import SetBool

# Enums
from .command_enums import ArmCommanderException


class TrajectoriesExecutor:
    """
    Object which execute the Arm trajectories via MoveIt
    """

    def __init__(self, arm_move_group):
        self.__arm = arm_move_group
        self.__joints_name = rospy.get_param('~joint_names')
        self.__hardware_version = rospy.get_param('~hardware_version')

        # - Direct topic to joint_trajectory_controller
        self.__current_goal_id = None
        self.__current_feedback = None
        self.__current_goal_result = GoalStatus.LOST
        self.__collision_detected = False
        self.__cancel_goal = False

        # Event which allows to timeout if trajectory take too long
        self.__traj_finished_event = threading.Event()

        # Others params
        self.__trajectory_minimum_timeout = rospy.get_param("~trajectory_minimum_timeout")
        self.__compute_plan_max_tries = rospy.get_param("~compute_plan_max_tries")
        self.__cartesian_path_eef_steps = rospy.get_param("~eef_step")
        self.__cartesian_path_jump_threshold = rospy.get_param("~jump_threshold")

        # - Subscribers
        joint_controller_base_name = rospy.get_param("~joint_controller_name")
        rospy.Subscriber('{}/follow_joint_trajectory/goal'.format(joint_controller_base_name),
                         FollowJointTrajectoryActionGoal, self.__callback_new_goal)

        rospy.Subscriber('{}/follow_joint_trajectory/result'.format(joint_controller_base_name),
                         FollowJointTrajectoryActionResult, self.__callback_goal_result)

        # collision detected by End Effector could be a real or fake collision.
        # In a movement, if a collision detected, that will be a real collision, without a movement, it will be fake
        rospy.Subscriber('/niryo_robot/hardware_interface/collision_detected',
                         Bool, self.__callback_collision_detected)

        # - Publishers
        self.__traj_goal_pub = rospy.Publisher('{}/follow_joint_trajectory/goal'.format(joint_controller_base_name),
                                               FollowJointTrajectoryActionGoal, queue_size=1)

        self.__joint_trajectory_publisher = rospy.Publisher('{}/command'.format(joint_controller_base_name),
                                                            JointTrajectory, queue_size=10)

        self.__reset_controller_service = rospy.ServiceProxy('/niryo_robot/joints_interface/steppers_reset_controller',
                                                             Trigger)

        self.__collision_detected_publisher = rospy.Publisher('/niryo_robot/collision_detected', Bool, queue_size=10)

        rospy.on_shutdown(self.cancel_goal)

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

    def __callback_collision_detected(self, msg):
        # The collision detected by EE is used only here. It just means if there is a movement.
        self.__collision_detected = msg.data

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

    # -- Executors
    def compute_and_execute_plan_to_target(self):
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

            if self.__hardware_version == 'ned':
                self.__reset_controller()
            rospy.loginfo("Arm commander - Send MoveIt trajectory to controller.")
            status, message = self.execute_plan(plan)

            if status != CommandStatus.SHOULD_RESTART:
                return status, message
            if tries >= self.__compute_plan_max_tries:
                rospy.logerr("Arm commander - Big failure from the controller. Try to restart the robot")
                return CommandStatus.SHOULD_RESTART, "Please restart the robot and try again."
            rospy.logwarn("Arm commander - Will retry to compute "
                          "& execute trajectory {} time(s)".format(self.__compute_plan_max_tries - tries))

    def __get_computed_plan(self):
        """
        Get computed plan from MoveIt
        :return: the computed plan if MoveIt succeed else None
        """
        plan = self.__arm.plan()

        # handle changes between noetic / noetic
        if isinstance(plan, RobotTrajectory):
            return None if not plan.joint_trajectory.points else plan
        else:
            success, trajectory, planning_time, error_code = plan
            if not success or not trajectory.joint_trajectory.points:
                return None
            else:
                return trajectory

    def compute_and_execute_cartesian_plan(self, list_poses, velocity_factor=1.0, acceleration_factor=1.0):
        """
        Compute a cartesian plan according to list_poses and then, execute it
        The robot will follow a straight line between each points
        If the plan is not validate, raise ArmCommanderException
        :param list_poses:
        :param velocity_factor:
        :param acceleration_factor:
        :return: status, message
        """
        if len(list_poses) == 0:
            return GoalStatus.REJECTED, "No Waypoints"

        try:
            plan = self.compute_cartesian_plan(list_poses)
        except Exception:
            raise ArmCommanderException(CommandStatus.ARM_COMMANDER_FAILURE, "IK Fail")

        if plan is None:
            raise ArmCommanderException(CommandStatus.NO_PLAN_AVAILABLE,
                                        "The goal cannot be reached with a linear trajectory")

        # Apply robot speeds
        plan = self.retime_plan(plan, velocity_scaling_factor=velocity_factor,
                                acceleration_scaling_factor=acceleration_factor, optimize=False)
        if plan is None:
            raise ArmCommanderException(CommandStatus.NO_PLAN_AVAILABLE,
                                        "The goal cannot be reached with a linear trajectory")

        return self.execute_plan(plan)

    def compute_cartesian_plan(self, list_poses, compute_max_tries=None):
        """
        Compute cartesian plan from a list of poses
        As 2 poses cannot be the same, the first operation is to filter list_poses to be sure this condition is met
        :param list_poses: list of Pose Object
        :param compute_max_tries: number of tries to compute the path, if None the default value will be taken
        :return: Computed plan : RobotTrajectory object
        """
        if len(list_poses) == 0:
            return None

        for i in reversed(range(len(list_poses) - 1)):
            if poses_too_close(list_poses[i + 1], list_poses[i]):
                list_poses.pop(i)

        fraction = 0.0
        for _ in range(compute_max_tries if compute_max_tries else self.__compute_plan_max_tries):  # some tries
            trajectory_plan, fraction = \
                self.__arm.compute_cartesian_path(list_poses, eef_step=self.__cartesian_path_eef_steps,
                                                  jump_threshold=self.__cartesian_path_jump_threshold)

            # Check the fraction value : if 1.0, the trajectory can be linear;
            # else, the trajectory followed won't be linear.
            if fraction == 1.0:
                # delete the very first joints position which is the starting position (current),
                # to avoid an error related to increasing time
                # del trajectory_plan.joint_trajectory.points[0]
                return trajectory_plan
        else:
            rospy.logwarn("Linear trajectory not found, only {}% was successfully computed.".format(fraction * 100))
            return None

    def execute_plan(self, plan):
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

        trajectory_time_out = max(1.5 * self.__get_plan_time(plan), self.__trajectory_minimum_timeout)

        self.__current_goal_id = None
        self.__current_goal_result = GoalStatus.LOST
        self.__cancel_goal = False

        # Send trajectory and wait
        self.__arm.execute(plan, wait=False)
        if self.__traj_finished_event.wait(trajectory_time_out):
            if self.__current_goal_result == GoalStatus.SUCCEEDED:
                return CommandStatus.SUCCESS, "Command has been successfully processed"
            elif self.__current_goal_result == GoalStatus.PREEMPTED:
                if self.__cancel_goal:
                    return CommandStatus.STOPPED, "Command has been successfully stopped"

                self.__collision_detected_publisher.publish(True)
                self.__set_learning_mode(True)
                msg = "Command has been aborted due to a collision or a motor not able to follow the given trajectory"
                return CommandStatus.COLLISION, msg

            elif self.__current_goal_result == GoalStatus.ABORTED:
                # if joint_trajectory_controller aborts the goal, it will still try to
                # finish executing the trajectory --> so we ask it to stop from here
                # http://wiki.ros.org/joint_trajectory_controller -> preemption policy
                # Send an empty trajectory from the topic interface
                self.__set_position_hold_mode()
                self.__set_learning_mode(True)
                self.__collision_detected_publisher.publish(True)
                msg = "Command has been aborted due to a collision or a motor not able to follow the given trajectory"
                return CommandStatus.COLLISION, msg

            else:  # problem from ros_control itself
                self.__current_goal_id = None
                return CommandStatus.SHOULD_RESTART, ""
        else:
            # This timeout will happen if something fails in ros_control
            # It is not related to a trajectory failure
            self.__current_goal_id = None
            return CommandStatus.SHOULD_RESTART, ""

    def plan(self):
        for _ in range(self.__compute_plan_max_tries):
            plan = self.__get_computed_plan()
            if plan is not None:
                return plan
        else:
            raise ArmCommanderException(CommandStatus.NO_PLAN_AVAILABLE, "No trajectory found.")

    def execute_joint_trajectory(self, joint_trajectory):
        """

        :param joint_trajectory:
        :type joint_trajectory: JointTrajectory
        :return:
        :rtype:
        """
        moveit_plan = RobotTrajectory(joint_trajectory=joint_trajectory)
        self.__arm.set_joint_value_target(moveit_plan.joint_trajectory.points[0].positions)
        partial_plan = self.plan()
        plan = self.link_plans(partial_plan, moveit_plan)
        return self.execute_plan(plan)

    # --- Callable functions
    def stop_arm(self):
        rospy.loginfo("Arm commander - Send STOP to arm")
        self.__arm.stop()

    def stop_current_plan(self):
        rospy.loginfo("Arm commander - Send STOP to arm and RESET to controller")
        self.__arm.stop()
        self.__reset_controller()

    def cancel_goal(self):
        self.__cancel_goal = True
        self.stop_current_plan()

    def retime_plan(self, plan, velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0, optimize=False):
        """
        Take a plan and retime it
        """
        start_state = self.__get_plan_start_robot_state(plan)
        if self.__hardware_version == "one":
            # for ros kinetic
            plan_out = self.__arm.retime_trajectory(start_state, plan, velocity_scaling_factor=velocity_scaling_factor)
        else:
            if optimize:
                algorithm = "time_optimal_trajectory_generation"
            else:
                algorithm = "iterative_time_parameterization"

            plan_out = self.__arm.retime_trajectory(start_state, plan,
                                                    velocity_scaling_factor=velocity_scaling_factor,
                                                    acceleration_scaling_factor=acceleration_scaling_factor,
                                                    algorithm=algorithm)
        return plan_out

    def __get_plan_start_robot_state(self, plan):
        start_state = RobotStateMoveIt()
        start_state.joint_state.header.frame_id = "world"
        start_state.joint_state.position = plan.joint_trajectory.points[0].positions
        start_state.joint_state.name = self.__joints_name
        return start_state

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

    def link_plans(self, *plans):
        # Link plans
        final_plan = plans[0]

        for plan in plans[1:]:
            final_plan.joint_trajectory.points.extend(plan.joint_trajectory.points)

        # Retime plan et recompute velocities
        final_plan = self.retime_plan(final_plan, optimize=True)
        return self.filtering_plan(final_plan)

    @staticmethod
    def filtering_plan(plan):
        if plan is None:
            return None

        new_plan = RobotTrajectory()
        new_plan.joint_trajectory.header = plan.joint_trajectory.header
        new_plan.joint_trajectory.joint_names = plan.joint_trajectory.joint_names
        new_plan.joint_trajectory.points = []

        for i, point in enumerate(plan.joint_trajectory.points[:-1]):
            if point.time_from_start != plan.joint_trajectory.points[i + 1].time_from_start:
                new_plan.joint_trajectory.points.append(point)
        new_plan.joint_trajectory.points.append(plan.joint_trajectory.points[-1])
        return new_plan

    def __set_learning_mode(self, set_bool):
        """
        Activate or deactivate the learning mode using the ros service /niryo_robot/learning_mode/activate

        :param set_bool:
        :type set_bool: bool

        :return: Success if the learning mode was properly activate or deactivate, False if not
        :rtype: bool
        """
        if set_bool and self.__hardware_version in ['ned2', 'ned3pro']:
            return True

        try:
            rospy.wait_for_service('/niryo_robot/learning_mode/activate', timeout=1)
            srv = rospy.ServiceProxy('/niryo_robot/learning_mode/activate', SetBool)
            resp = srv(set_bool)
            return resp.status == CommandStatus.SUCCESS
        except (rospy.ServiceException, rospy.ROSException):
            return False

    @staticmethod
    def display_traj(point_list, id_=1):
        topic_display = 'visualization_marker_array'
        if topic_display in rospy.get_published_topics():
            markers_array = rospy.wait_for_message(topic_display, MarkerArray).markers
        else:
            markers_array = []

        marker_pub = rospy.Publisher(topic_display, MarkerArray, queue_size=10, latch=True)

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

        cardboard_marker.pose.orientation = Quaternion(0, 0, 0, 1)
        cardboard_marker.points = point_list
        cardboard_marker.lifetime = rospy.Duration(0)
        markers_array.append(cardboard_marker)
        marker_pub.publish(markers_array)
