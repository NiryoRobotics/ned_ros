#!/usr/bin/env python

# Libs
import rospy

from transform_handler import PosesTransformHandler
from niryo_robot_poses_handlers.transform_functions import euler_from_quaternion
from niryo_robot_poses_handlers.file_manager import NiryoRobotFileException
from niryo_robot_poses_handlers.grip_manager import GripManager
from niryo_robot_poses_handlers.pose_manager import PoseManager
from niryo_robot_poses_handlers.trajectory_manager import TrajectoryManager
from niryo_robot_poses_handlers.workspace_manager import WorkspaceManager

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from niryo_robot_msgs.msg import RobotState
from niryo_robot_msgs.msg import RPY
from niryo_robot_poses_handlers.msg import NiryoPose
from std_msgs.msg import Int32
from niryo_robot_tools_commander.msg import TCP

# Services
from niryo_robot_msgs.srv import GetNameDescriptionList

from niryo_robot_poses_handlers.srv import GetTargetPose
from niryo_robot_poses_handlers.srv import GetWorkspaceRatio
from niryo_robot_poses_handlers.srv import GetWorkspaceRobotPoses
from niryo_robot_poses_handlers.srv import ManageWorkspace
from niryo_robot_poses_handlers.srv import GetPose, ManagePose
from niryo_robot_poses_handlers.srv import GetTrajectory, ManageTrajectory


class PoseHandlerNode:
    """
    This class is an interface for manipulating object poses and converting them to robot poses.
    In particular, it allows
        - creation of workspaces
        - created of grips
        - convertion of workspace-relative poses (e.g. from camera) to robot poses
    """

    def __init__(self):
        rospy.logdebug("Poses Handlers - Entering in Init")

        # Subscribers
        self.__tcp_enabled = False
        rospy.Subscriber('/niryo_robot_tools_commander/tcp', TCP, self.__callback_tcp)

        self.__tool_id = 0
        rospy.Subscriber('/niryo_robot_tools_commander/current_id', Int32,  self.__callback_tool_id)


        # Workspaces
        ws_dir = rospy.get_param("~workspace_dir")
        self.__ws_manager = WorkspaceManager(ws_dir)
        rospy.Service('~manage_workspace', ManageWorkspace,
                      self.__callback_manage_workspace)
        rospy.Service('~get_workspace_ratio', GetWorkspaceRatio,
                      self.__callback_workspace_ratio)
        rospy.Service('~get_workspace_list', GetNameDescriptionList,
                      self.__callback_workspace_list)
        rospy.Service('~get_workspace_poses', GetWorkspaceRobotPoses,
                      self.__callback_get_workspace_poses)

        if rospy.has_param('~gazebo_workspaces'):
            for ws_name, ws_poses in rospy.get_param('~gazebo_workspaces').items():
                if ws_name not in self.get_available_workspaces()[0]:
                    rospy.loginfo("Poses Handler - Adding the {} workspace...".format(ws_name))
                    self.create_workspace_from_points(ws_name, "", [Point(*point) for point in ws_poses])


        # Grips
        tool_config_dict = rospy.get_param("niryo_robot_tools_commander/tool_list", dict())
        self.__tool_id_gripname_dict = {tool["id"]: "default_" + tool["name"].replace(" ", "_")
                                        for tool in tool_config_dict}
        self.__tool_id_gripname_dict[-1] = "default_Calibration_Tip"

        grip_dir = rospy.get_param("~grip_dir")
        self.__grip_manager = GripManager(grip_dir, self.__tool_id_gripname_dict.values())
        rospy.Service('~get_target_pose', GetTargetPose,
                      self.__callback_target_pose)

        # Transform Handlers
        self.__transform_handler = PosesTransformHandler(self.__grip_manager)

        rospy.logdebug("Poses Handlers - Transform Handler created")

        # -- POSES AND TRAJECTORIES
        # Poses
        poses_dir = rospy.get_param("~poses_dir")
        self.__pos_manager = PoseManager(poses_dir)

        rospy.Service('~manage_pose', ManagePose,
                      self.__callback_manage_pose)
        rospy.Service('~get_pose', GetPose,
                      self.__callback_get_pose)
        rospy.Service('~get_pose_list', GetNameDescriptionList,
                      self.__callback_get_position_list)

        # Trajectories
        trajectories_dir = rospy.get_param("~trajectories_dir")
        self.traj_manager = TrajectoryManager(trajectories_dir)
        rospy.Service('~manage_trajectory', ManageTrajectory,
                      self.__callback_manage_trajectory)
        rospy.Service('~get_trajectory', GetTrajectory,
                      self.__callback_get_trajectory)
        rospy.Service('~get_trajectory_list', GetNameDescriptionList,
                      self.__callback_get_trajectory_list)

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.loginfo("Poses Handlers - Started")

    # -- ROS CALLBACKS
    def __callback_tcp(self, msg):
        self.__tcp_enabled = msg.enabled

    def __callback_tool_id(self, msg):
        self.__tool_id = msg.data

    # Workspace
    def __callback_manage_workspace(self, req):
        cmd = req.cmd
        workspace = req.workspace
        if len(workspace.name)>30:
            rospy.logwarn('Poses Handlers - Workspace name is too long, using : %s instead', workspace.name[:30])
        workspace.name = workspace.name[:30]
        if cmd == req.SAVE:
            if len(workspace.poses) != 4:
                return CommandStatus.WORKSPACE_CREATION_FAILED, "Workspaces have 4 positions, {} given".format(
                    len(workspace.poses))
            try:
                self.create_workspace(workspace.name, workspace.description, workspace.poses)
                return CommandStatus.SUCCESS, "Created workspace '{}'".format(workspace.name)
            except NiryoRobotFileException as e:
                return CommandStatus.FILE_ALREADY_EXISTS, str(e)
            except (ValueError, Exception) as e:
                return CommandStatus.WORKSPACE_CREATION_FAILED, str(e)

        elif cmd == req.SAVE_WITH_POINTS:
            if len(workspace.points) != 4:
                return CommandStatus.WORKSPACE_CREATION_FAILED, "Workspaces have 4 points, {} given".format(
                    len(workspace.points))
            try:
                self.create_workspace_from_points(workspace.name, workspace.description, workspace.points)
                return CommandStatus.SUCCESS, "Created workspace '{}'".format(workspace.name)
            except NiryoRobotFileException as e:
                return CommandStatus.FILE_ALREADY_EXISTS, str(e)
            except Exception as e:
                return CommandStatus.WORKSPACE_CREATION_FAILED, str(e)
        elif cmd == req.DELETE:
            try:
                self.remove_workspace(workspace.name)
                return CommandStatus.SUCCESS, "Removed workspace '{}'".format(workspace.name)
            except Exception as e:
                return CommandStatus.POSES_HANDLER_REMOVAL_FAILED, str(e)
        else:
            return CommandStatus.UNKNOWN_COMMAND, "cmd '{}' not found.".format(req.cmd)

    def __callback_workspace_ratio(self, req):
        try:
            ratio = self.get_workspace_ratio(req.workspace)
            return CommandStatus.SUCCESS, "Success", ratio
        except Exception as e:
            return CommandStatus.POSES_HANDLER_READ_FAILURE, str(e), 0

    def __callback_workspace_list(self, _):
        try:
            ws_list, description_list = self.get_available_workspaces()
        except Exception as e:
            rospy.logerr("Poses Handlers - Error occured when getting workspace list: {}".format(e))
            ws_list = description_list = []
        return {"name_list": ws_list, "description_list": description_list}

    def __callback_get_workspace_poses(self, req):
        try:
            poses = self.get_workspace_poses(req.name)
            return CommandStatus.SUCCESS, "Success", poses
        except Exception as e:
            return CommandStatus.POSES_HANDLER_READ_FAILURE, str(e), 4 * [RobotState()]

    # Grips
    def __callback_target_pose(self, req):
        try:
            pose = self.get_target_pose(req.workspace, req.height_offset, req.x_rel, req.y_rel, req.yaw_rel)
            return CommandStatus.SUCCESS, "Success", pose
        except Exception as e:
            return CommandStatus.POSES_HANDLER_COMPUTE_FAILURE, str(e), RobotState()

    # Position
    def __callback_get_position_list(self, _):
        try:
            pos_list, description_list = self.get_available_poses()
        except Exception as e:
            rospy.logerr("Poses Handlers - Error occured when getting position list: {}".format(e))
            pos_list = description_list = []
        return {"name_list": pos_list, "description_list": description_list}

    def __callback_get_pose(self, req):
        try:
            pos_read = self.get_pose(req.name)
            pos_msg = NiryoPose(pos_read.name, pos_read.description,
                                pos_read.joints, pos_read.position, pos_read.rpy, pos_read.orientation)
            return CommandStatus.SUCCESS, "Success", pos_msg
        except Exception as e:
            return CommandStatus.POSES_HANDLER_READ_FAILURE, str(e), NiryoPose()

    def __callback_manage_pose(self, req):
        cmd = req.cmd
        pose = req.pose
        if cmd == req.SAVE:
            try:
                self.create_pose(pose.name, pose.description, pose.joints, pose.position, pose.rpy, pose.orientation)
                return CommandStatus.SUCCESS, "Created Position '{}'".format(pose.name)
            except Exception as e:
                return CommandStatus.POSES_HANDLER_CREATION_FAILED, str(e)
        elif cmd == req.DELETE:
            try:
                self.remove_pose(pose.name)
                return CommandStatus.SUCCESS, "Removed Position '{}'".format(pose.name)
            except Exception as e:
                return CommandStatus.POSES_HANDLER_REMOVAL_FAILED, str(e)
        else:
            return CommandStatus.UNKNOWN_COMMAND, "cmd '{}' not found.".format(cmd)

    def __callback_manage_trajectory(self, req):
        cmd = req.cmd
        if cmd == req.SAVE:
            try:
                self.create_trajectory(req.name, req.description, req.poses)
                return CommandStatus.SUCCESS, "Created trajectory '{}'".format(req.name)
            except Exception as e:
                return CommandStatus.POSES_HANDLER_CREATION_FAILED, str(e)
        elif cmd == req.DELETE:
            try:
                self.remove_trajectory(req.name)
                return CommandStatus.SUCCESS, "Removed trajectory '{}'".format(req.name)
            except Exception as e:
                return CommandStatus.POSES_HANDLER_REMOVAL_FAILED, str(e)
        else:
            return CommandStatus.UNKNOWN_COMMAND, "cmd '{}' not found.".format(req.cmd)

    # Trajectories
    def __callback_get_trajectory_list(self, _):
        try:
            trajectory_list, description_list = self.get_available_trajectories_w_description()
        except Exception as e:
            rospy.logerr("Poses Handlers - Error occured when getting trajectories list: {}".format(e))
            trajectory_list = description_list = []
        return {"name_list": trajectory_list, "description_list": description_list}

    def __callback_get_trajectory(self, req):
        try:
            traj_list_poses = self.get_trajectory(req.name)
            return CommandStatus.SUCCESS, "Success", traj_list_poses
        except Exception as e:
            return CommandStatus.POSES_HANDLER_READ_FAILURE, str(e), []

    # -- REGULAR CLASS FUNCTIONS
    # Workspace
    def create_workspace(self, name, description, robot_poses):
        """
        Creates a new workspace based on 4 recorded robot poses. The 0th point
        needs to be the workspace origin. Points ordered clockwise.

        :param name: name of the new workspace
        :param description: description of the new workspace
        :param robot_poses: [pose when pointing at origin, pose on marker 1,...]
        """
        points = []
        robot_poses_raw = []

        for pose in robot_poses:
            rospy.logdebug("Robot point\n{}".format(pose.position))
            point = self.__transform_handler.get_calibration_tip_position(pose)
            rospy.logdebug("Tip point\n{}".format(point))
            points.append([point.x, point.y, point.z])
            pose_raw = [[pose.position.x, pose.position.y, pose.position.z],
                        [pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw]]
            robot_poses_raw.append(pose_raw)
        self.__ws_manager.create(name, points, robot_poses_raw, description)

    def create_workspace_from_points(self, name, description, points):
        """

        :param name: name of the new workspace
        :param description: description of the new workspace
        :param points: [pose of origin marker, pose of marker 1,...]
        """
        points_raw = []
        for point in points:
            points_raw.append([point.x, point.y, point.z])
        self.__ws_manager.create(name, points_raw, description)

    def remove_workspace(self, name):
        """
        Removes a workspace
        :param name: name of the workspace to remove
        """
        self.__ws_manager.remove(name)

    def get_available_workspaces(self):
        """
        Returns a list of all available workspace names.
        """
        return self.__ws_manager.get_all_names_w_description()

    def get_workspace_poses(self, name):
        ws_raw = self.__ws_manager.read(name)
        poses = [RobotState(position=Point(*pose[0]), rpy=RPY(*pose[1]))
                 for pose in ws_raw.robot_poses]
        return poses

    def get_workspace_ratio(self, workspace):
        """
        Reads the ratio (width/height) of the specified workspace. This function is used by the
        camera to do the perspective transform.

        :param workspace: name of the workspace
        """
        current_ws = self.__ws_manager.read(workspace)
        return current_ws.x_width / current_ws.y_width

    # Grips
    def get_target_pose(self, workspace, height_offset, x_rel, y_rel, yaw_rel):
        """
        Computes the robot pose that can be used to grab an object which is
        positioned relative to the given workspace.

        :param workspace: name of the workspace the object is in
        :param height_offset: z-offset that is added
        :param x_rel: x relative position of the object inside working zone
        :param y_rel: y relative position of the object inside working zone
        :param yaw_rel: angle of the object inside working zone
        """
        current_ws = self.__ws_manager.read(workspace)
        self.__transform_handler.set_relative_pose_object(current_ws, x_rel, y_rel, yaw_rel,
                                                          yaw_center=current_ws.yaw_center)
        if self.__tcp_enabled:
            base_link_to_tool_target = self.__transform_handler.get_object_transform(z_off=height_offset)
        else:
            current_grip = self.__grip_manager.read(self.__tool_id_gripname_dict[self.__tool_id])
            current_grip.transform.transform.translation.z += height_offset
            self.__transform_handler.set_grip(current_grip)
            base_link_to_tool_target = self.__transform_handler.get_gripping_transform()

        q = base_link_to_tool_target.transform.rotation

        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        pose = RobotState()
        pose.position.x = base_link_to_tool_target.transform.translation.x
        pose.position.y = base_link_to_tool_target.transform.translation.y
        pose.position.z = base_link_to_tool_target.transform.translation.z
        pose.rpy.roll = roll
        pose.rpy.pitch = pitch
        pose.rpy.yaw = yaw

        return pose

    # -- Pose
    def create_pose(self, name, description, joints, point, rpy, quaternion):
        """
        Create a pose from ManagePose message fields

        :type name: str
        :type description: str
        :type joints: list[float]
        :type point: Point
        :type rpy: RPY
        :type quaternion: Quaternion
        :return: None
        """
        point_raw = [point.x, point.y, point.z]
        rpy_raw = [rpy.roll, rpy.pitch, rpy.yaw]
        quaternion_raw = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        self.__pos_manager.create(name, joints, point_raw, rpy_raw, quaternion_raw, description)

    def get_pose(self, name):
        """
        Get pose from pose manager and return NiryoPose

        :param name: pose name
        :type name: str
        :return: The pose object
        :rtype: NiryoPose
        """
        pose_raw = self.__pos_manager.read(name)
        pose = NiryoPose()
        pose.joints = pose_raw.joints
        pose.position = Point(*pose_raw.position)
        pose.rpy = RPY(*pose_raw.rpy)
        pose.orientation = Quaternion(*pose_raw.orientation)
        return pose

    def remove_pose(self, name):
        """
        Asks pose manager to remove pose

        :param name: pose name
        :type name: str
        :return: None
        """
        self.__pos_manager.remove(name)

    def get_available_poses(self):
        """
        Ask the poses manager which pose(s) are available

        :return: list of positions name
        :rtype: list[str]
        """
        return self.__pos_manager.get_all_names_w_description()

    # -- Trajectories
    def create_trajectory(self, name, description, poses):
        """
        Create a pose from ManageTrajectory message fields

        :type name: str
        :type description: str
        :type poses: list[Pose]
        :return: None
        """
        poses_raw = []
        for pose in poses:
            point_raw = [pose.position.x, pose.position.y, pose.position.z]
            quaternion_raw = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            pose_raw = [point_raw, quaternion_raw]
            poses_raw.append(pose_raw)
        self.traj_manager.create(name, poses_raw, description)

    def get_trajectory(self, name):
        """
        Get trajectory from trajectories manager and return list of NiryoPose

        :param name: pose name
        :type name: str
        :return: The trajectory object
        :rtype: list[NiryoPose]
        """
        traj_read = self.traj_manager.read(name)
        list_poses_raw = traj_read.list_poses
        list_poses = []
        for pose_raw in list_poses_raw:
            point = Point(*pose_raw[0])
            quaternion = Quaternion(*pose_raw[1])
            pose = Pose(point, quaternion)
            list_poses.append(pose)
        return list_poses

    def remove_trajectory(self, name):
        """
        Asks trajectories manager to remove pose

        :param name: trajectory name
        :type name: str
        :return: None
        """
        self.traj_manager.remove(name)

    def get_available_trajectories_w_description(self):
        """
        Ask the trajectories manager which trajectories are available

        :return: list of trajectories name
        :rtype: list[str]
        """
        return self.traj_manager.get_all_names_w_description()


if __name__ == "__main__":
    rospy.init_node('niryo_robot_poses_handlers', anonymous=False, log_level=rospy.INFO)
    try:
        vision_node = PoseHandlerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
