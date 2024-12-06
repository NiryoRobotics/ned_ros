#!/usr/bin/env python

import rospy
from threading import Lock, Thread

from niryo_robot_msgs.msg import CommandStatus, BasicObject, BasicObjectArray
from niryo_robot_msgs.srv import GetNameDescriptionList, GetNameDescriptionListResponse

from std_msgs.msg import Int32, Bool, Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from end_effector_interface.msg import EEButtonStatus

from .trajectory_file_manager import TrajectoryFileManager
from .command_enums import ArmCommanderException
from niryo_robot_arm_commander.srv import ManageTrajectory, ManageTrajectoryRequest, GetTrajectory

from niryo_robot_poses_handlers.file_manager import NiryoRobotFileException

EXECUTE_AFTER_LEARNED = False


class TrajectoryHandlerNode:

    def __init__(self, arm_state, trajectory_executor):
        self.__arm_state = arm_state
        self.__traj_executor = trajectory_executor

        self.__frequency = rospy.Rate(rospy.get_param("~trajectory_frequency"))
        self.__lock = Lock()
        self.__record_thread = Thread()
        self.__recording = False

        # - Subscribers
        self.__save_pos_button_topic = rospy.Subscriber(
            '/niryo_robot_hardware_interface/end_effector_interface/save_pos_button_status',
            EEButtonStatus,
            self.__callback_save_pos_button_status)

        # - Publisher
        self.save_trajectory_publisher = rospy.Publisher("/niryo_robot/blockly/save_trajectory", Int32, queue_size=10)
        self.__learning_trajectory_publisher = rospy.Publisher('~learning_trajectory', Bool, queue_size=10)
        self.__trajectory_list_publisher = rospy.Publisher('~trajectory_list',
                                                           BasicObjectArray,
                                                           queue_size=10,
                                                           latch=True)

        # Services
        self.traj_file_manager = TrajectoryFileManager(rospy.get_param("/niryo_robot_poses_handlers/trajectories_dir"))
        rospy.Service('~manage_trajectory', ManageTrajectory, self.__callback_manage_trajectory)
        rospy.Service('~get_trajectory', GetTrajectory, self.__callback_get_trajectory)
        rospy.Service('~get_trajectory_list', GetNameDescriptionList, self.__callback_get_trajectory_list)

        rospy.on_shutdown(self.stop_record)

        self.__publish_trajectory_list()

    def save_trajectory(self, trajectory, trajectory_name="last_executed_trajectory", description="", auto=True):
        if self.check_trajectory_existence(trajectory_name) and not auto:
            return False
        self.create_trajectory_file(str(trajectory_name), str(description), trajectory.points)
        return True

    def update_trajectory(self, name="", description="", new_name=""):
        status = True
        if name:
            if self.check_trajectory_existence(name):
                if (not self.check_trajectory_existence(new_name) and name != new_name) or (name == new_name):
                    try:
                        trajectory = self.get_trajectory(name)
                        self.remove_trajectory_file(name)
                        self.create_trajectory_file(new_name, description, trajectory.points)
                    except Exception:
                        rospy.logwarn("TrajectoryHandlerNode::update_trajectory - Wrong trajectory format")
                        status = False
                else:
                    status = False
            else:
                status = False
        else:
            status = False
        return status

    def check_trajectory_existence(self, trajectory_name):
        trajectory_name_list, _description_list = self.get_available_trajectories_w_description()
        if str(trajectory_name) in trajectory_name_list:
            return True
        else:
            return False

    def create_trajectory_file(self, name, description, points):
        """
        Create a pose from ManageTrajectory message fields

        :type name: str
        :type description: str
        :type points: list[JointTrajectoryPoint]
        :return: None
        """
        self.traj_file_manager.create(str(name), points, str(description))
        self.__publish_trajectory_list()

    def get_trajectory(self, name):
        """
        Get trajectory from trajectories manager and return a JointTrajectory

        :param name: pose name
        :type name: str
        :return: The trajectory object
        :rtype: JointTrajectory
        """
        traj_read = self.traj_file_manager.read(str(name))
        list_poses_raw = traj_read.list_poses
        return JointTrajectory(header=Header(stamp=rospy.Time.now()),
                               joint_names=rospy.get_param("~joint_names"),
                               points=[JointTrajectoryPoint(positions=pose_raw) for pose_raw in list_poses_raw])

    def get_trajectory_first_point(self, name):
        """
        Get trajectory's from trajectories manager and return list of NiryoPose

        :param name: pose name
        :type name: str
        :return: The trajectory's first pose object
        :rtype: Joints
        """
        try:
            trajectory = self.get_trajectory(name)
            return trajectory.points[0].positions
        except Exception as e:
            rospy.logwarn(f'TrajectoryHandlerNode::get_trajectory_first_point - {e}')

    def get_trajectory_file(self, name):
        """
        Get the trajectory file associated to the name given

        :param name: pose name
        :type name: str
        :return: The trajectory file
        :rtype: dict
        """
        try:
            return self.traj_file_manager.read(str(name))
        except Exception as e:
            rospy.logwarn(f'TrajectoryHandlerNode::get_trajectory_file - {e}')

    def remove_trajectory_file(self, name):
        """
        Asks trajectories manager to remove pose

        :param name: trajectory name
        :type name: str
        :return: None
        """
        self.traj_file_manager.remove(str(name))
        self.__publish_trajectory_list()

    def get_available_trajectories_w_description(self):
        """
        Ask the trajectories' manager which trajectories are available

        :return: list of trajectories name
        :rtype: list[str]
        """
        return self.traj_file_manager.get_all_names_w_description()

    def record(self):
        if self.__lock.acquire(False):
            try:
                self.__recording = True
                self.__learning_trajectory_publisher.publish(True)
                self.__arm_state.force_learning_mode(True)

                trajectory = JointTrajectory(header=Header(), joint_names=rospy.get_param("~joint_names"), points=[])
                time_ref = rospy.Time.now()
                while not rospy.is_shutdown() and self.__recording:
                    trajectory.points.append(
                        JointTrajectoryPoint(time_from_start=rospy.Time.now() - time_ref,
                                             positions=self.__arm_state.joint_states))
                    self.__frequency.sleep()
                self.__recording = False
                self.__learning_trajectory_publisher.publish(False)
                self.__arm_state.force_learning_mode(False)
            finally:
                self.__lock.release()
        else:
            return

        if len(trajectory.points) > 1:
            self.save_trajectory(trajectory)
            self.blockly_save_trajectory()
            try:
                result = self.get_trajectory("last_executed_trajectory")
                if EXECUTE_AFTER_LEARNED:
                    self.__traj_executor.execute_joint_trajectory(result)
            except Exception:
                rospy.logwarn("TrajectoryHandlerNode::record - Wrong trajectory format")
        else:
            print("Not enough points to create Trajectory")

    def stop_record(self):
        self.__recording = False

    # - Callbacks

    def __callback_manage_trajectory(self, req):
        cmd = req.cmd
        if cmd == req.SAVE_LAST_LEARNED:
            try:
                if not self.check_trajectory_existence("last_executed_trajectory"):
                    return CommandStatus.TRAJECTORY_HANDLER_RENAME_FAILURE, \
                        "No trajectory to save on hold"

                status = self.update_trajectory("last_executed_trajectory", req.description, req.name)
                if status:
                    return CommandStatus.SUCCESS, "Created trajectory '{}'".format(req.name)
                else:
                    return CommandStatus.TRAJECTORY_HANDLER_RENAME_FAILURE, \
                        "A trajectory is already saved with this name"
            except NiryoRobotFileException as e:
                return CommandStatus.TRAJECTORY_HANDLER_RENAME_FAILURE, str(e)
        elif cmd == req.SAVE:
            try:
                _status = self.save_trajectory(req.trajectory, req.name, req.description)
                if _status:
                    return CommandStatus.SUCCESS, "Saved trajectory '{}'".format(req.name)
                else:
                    return CommandStatus.TRAJECTORY_HANDLER_CREATION_FAILED, 'Name already taken'
            except NiryoRobotFileException as e:
                return CommandStatus.TRAJECTORY_HANDLER_CREATION_FAILED, str(e)
        elif cmd == req.DELETE:
            try:
                self.remove_trajectory_file(req.name)
                return CommandStatus.SUCCESS, "Removed trajectory '{}'".format(req.name)
            except NiryoRobotFileException as e:
                return CommandStatus.TRAJECTORY_HANDLER_REMOVAL_FAILED, str(e)
        elif cmd == req.DELETE_ALL:
            try:
                trajectory_name_list, description_list = self.get_available_trajectories_w_description()
                for trajectory in trajectory_name_list:
                    self.remove_trajectory_file(trajectory)
                return CommandStatus.SUCCESS, "Removed All Trajectories "
            except NiryoRobotFileException as e:
                return CommandStatus.TRAJECTORY_HANDLER_REMOVAL_FAILED, str(e)
        elif cmd == req.UPDATE:
            try:
                status = self.update_trajectory(req.name, req.description, req.new_name)
                if status:
                    return CommandStatus.SUCCESS, "Updated Trajectory '{}'".format(req.name)
                else:
                    return CommandStatus.TRAJECTORY_HANDLER_RENAME_FAILURE, "Name already taken, Update Failed"
            except NiryoRobotFileException as e:
                return CommandStatus.TRAJECTORY_HANDLER_RENAME_FAILURE, str(e)
        elif cmd == req.EXECUTE_REGISTERED:
            try:
                if self.check_trajectory_existence(req.name):
                    trajectory = self.get_trajectory(req.name)
                    return self.__traj_executor.execute_joint_trajectory(trajectory)
                else:
                    return CommandStatus.TRAJECTORY_HANDLER_EXECUTE_REGISTERED_FAILURE, str(NiryoRobotFileException)
            except ArmCommanderException as e:
                return CommandStatus.TRAJECTORY_HANDLER_EXECUTE_REGISTERED_FAILURE, str(e)
        elif cmd == req.EXECUTE:
            try:
                return self.__traj_executor.execute_joint_trajectory(req.trajectory)
            except ArmCommanderException as e:
                return CommandStatus.TRAJECTORY_HANDLER_EXECUTE_FAILURE, str(e)
        elif cmd == req.GO_TO_FIRST_POINT:
            try:
                trajectory = self.get_trajectory(req.name)
                trajectory.points = [trajectory.points[0]]
                return self.__traj_executor.execute_joint_trajectory(trajectory)
            except ArmCommanderException as e:
                return CommandStatus.TRAJECTORY_HANDLER_EXECUTE_FAILURE, str(e)
        else:
            return CommandStatus.UNKNOWN_COMMAND, "cmd '{}' not found.".format(req.cmd)

    def __callback_get_trajectory_list(self, _):
        response = GetNameDescriptionListResponse()
        response.status = CommandStatus.SUCCESS

        try:
            response.name_list, response.description_list = self.get_available_trajectories_w_description()
        except Exception as e:
            rospy.logerr("Trajectory Handlers - Error occured when getting trajectories list: {}".format(e))
            response.status = CommandStatus.POSES_HANDLER_READ_FAILURE
            response.message = str(e)

        response.objects = [
            BasicObject(name=name, description=description) for name,
            description in zip(response.name_list, response.description_list)
        ]
        return response

    def __callback_get_trajectory(self, req):
        try:
            trajectory = self.get_trajectory(req.name)
            return CommandStatus.SUCCESS, "Success", trajectory
        except NiryoRobotFileException as e:
            return CommandStatus.TRAJECTORY_HANDLER_GET_TRAJECTORY_FAILURE, str(e), JointTrajectory()

    def __callback_save_pos_button_status(self, msg):
        if msg.action == EEButtonStatus.NO_ACTION:
            self.stop_record()

        elif msg.action == EEButtonStatus.LONG_PUSH_ACTION and not self.__recording:
            if not self.__record_thread.is_alive():
                self.__record_thread = Thread(target=self.record)
                self.__record_thread.start()

    def __publish_trajectory_list(self):
        trajectory_array = BasicObjectArray()
        trajectory_array.objects = [
            BasicObject(name=name, description=description) for name,
            description in zip(*self.get_available_trajectories_w_description())
        ]
        self.__trajectory_list_publisher.publish(trajectory_array)

    def blockly_save_trajectory(self):
        self.save_trajectory_publisher.publish(Int32(data=1))
