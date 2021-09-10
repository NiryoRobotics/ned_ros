#!/usr/bin/env python

# Lib
import rospy

import threading
import random

from niryo_robot_arm_commander.utils import poses_too_close

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryActionResult
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from moveit_msgs.msg import RobotState as RobotStateMoveIt
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion

# Services
from niryo_robot_msgs.srv import Trigger
from niryo_robot_msgs.srv import SetBool

# Enums
from niryo_robot_arm_commander.command_enums import ArmCommanderException


class TrajectoriesExecutor:
    """
    Object which execute the Arm trajectories via MoveIt
    """

    def __init__(self, arm_move_group):
        self.__arm = arm_move_group
        self.__joints_name = rospy.get_param('~joint_names')

        # - Direct topic to joint_trajectory_controller
        self.__current_goal_id = None
        self.__current_feedback = None
        self.__current_goal_result = GoalStatus.LOST
        self.__collision_detected = False

        # Event which allows to timeout if trajectory take too long
        self.__traj_finished_event = threading.Event()

        # Others params
        self.__trajectory_minimum_timeout = rospy.get_param("~trajectory_minimum_timeout")
        self.__compute_plan_max_tries = rospy.get_param("~compute_plan_max_tries")
        self.__error_tolerance = rospy.get_param("~error_tolerance")
        self.__cartesian_path_eef_steps = rospy.get_param("~eef_step")
        self.__cartesian_path_jump_threshold = rospy.get_param("~jump_threshold")

        # - Subscribers
        joint_controller_base_name = rospy.get_param("~joint_controller_name")
        rospy.Subscriber('{}/follow_joint_trajectory/goal'.format(joint_controller_base_name),
                         FollowJointTrajectoryActionGoal, self.__callback_new_goal)

        rospy.Subscriber('{}/follow_joint_trajectory/result'.format(joint_controller_base_name),
                         FollowJointTrajectoryActionResult, self.__callback_goal_result)

        rospy.Subscriber('{}/follow_joint_trajectory/feedback'.format(joint_controller_base_name),
                         FollowJointTrajectoryActionFeedback, self.__callback_current_feedback)

        # - Publishers
        self.__traj_goal_pub = rospy.Publisher('{}/follow_joint_trajectory/goal'.format(joint_controller_base_name),
                                               FollowJointTrajectoryActionGoal, queue_size=1)

        self.__joint_trajectory_publisher = rospy.Publisher('{}/command'.format(joint_controller_base_name),
                                                            JointTrajectory, queue_size=10)
        self.__reset_controller_service = rospy.ServiceProxy('/niryo_robot/joints_interface/steppers_reset_controller',
                                                             Trigger)

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

    def __callback_current_feedback(self, msg):
        self.__current_feedback = msg
        self.__collision_detected = False
        for error, tolerance in zip(self.__current_feedback.feedback.error.positions, self.__error_tolerance):
            if abs(error) > tolerance:
                self.__collision_detected = True
                self.__set_learning_mode(True)
                abort_str = "Command has been aborted due to a collision or " \
                            "a motor not able to follow the given trajectory"
                rospy.logwarn(abort_str)
                return CommandStatus.CONTROLLER_PROBLEMS, abort_str

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

            self.__reset_controller()
            rospy.logdebug("Arm commander - Send MoveIt trajectory to controller.")
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
        return None if not plan.joint_trajectory.points else plan

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
        self.__current_goal_id = None
        self.__current_goal_result = GoalStatus.LOST

        trajectory_time_out = max(1.5 * self.__get_plan_time(plan), self.__trajectory_minimum_timeout)

        # Send trajectory and wait
        self.__arm.execute(plan, wait=False)
        if self.__traj_finished_event.wait(trajectory_time_out):
            if self.__current_goal_result == GoalStatus.SUCCEEDED:
                return CommandStatus.SUCCESS, "Command has been successfully processed"
            elif self.__current_goal_result == GoalStatus.PREEMPTED:
                if self.__collision_detected:
                    self.__collision_detected = False
                    return CommandStatus.STOPPED, "Command has been aborted due to a collision or " \
                                                  "a motor not able to follow the given trajectory"
                else:
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

    def plan(self):
        for _ in range(self.__compute_plan_max_tries):
            partial_plan = self.__arm.plan()
            if len(partial_plan.joint_trajectory.points) != 0:
                return partial_plan
        else:
            raise ArmCommanderException(CommandStatus.NO_PLAN_AVAILABLE, "No trajectory found.")

    # --- Callable functions
    def stop_arm(self):
        rospy.loginfo("Arm commander - Send STOP to arm")
        self.__arm.stop()

    def stop_current_plan(self):
        rospy.loginfo("Arm commander - Send STOP to arm and RESET to controller")
        self.__arm.stop()
        self.__reset_controller()

    def retime_plan(self, plan, velocity_scaling_factor=1.0, acceleration_scaling_factor=1.0, optimize=False):
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

    @staticmethod
    def __set_learning_mode(set_bool):
        """
        Activate or deactivate the learning mode using the ros service /niryo_robot/learning_mode/activate

        :param set_bool:
        :type set_bool: bool

        :return: Success if the learning mode was properly activate or deactivate, False if not
        :rtype: bool
        """
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
