#!/usr/bin/env python3

# Libs
import rospy
import logging
import tf
import tf2_ros
import tf2_geometry_msgs

from niryo_robot_utils import sentry_init

from niryo_robot_poses_handlers.transform_handler import PosesTransformHandler
from niryo_robot_poses_handlers.transform_functions import euler_from_quaternion
from niryo_robot_poses_handlers.file_manager import NiryoRobotFileException
from niryo_robot_poses_handlers.grip_manager import GripManager
from niryo_robot_poses_handlers.pose_manager import PoseManager, PoseObj
from niryo_robot_poses_handlers.workspace_manager import WorkspaceManager
from niryo_robot_poses_handlers.dynamic_frame_manager import DynamicFrameManager
from niryo_robot_poses_handlers.transform_functions import convert_dh_convention_to_legacy_rpy

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from niryo_robot_msgs.msg import RobotState
from niryo_robot_msgs.msg import RPY
from niryo_robot_msgs.msg import BasicObject, BasicObjectArray

from niryo_robot_poses_handlers.msg import NiryoPose, DynamicFrame
from std_msgs.msg import Int32
from niryo_robot_tools_commander.msg import TCP

# Services
from niryo_robot_msgs.srv import GetNameDescriptionList, GetNameDescriptionListResponse, SetString

from niryo_robot_poses_handlers.srv import GetTargetPose, GetTransformPose
from niryo_robot_poses_handlers.srv import GetWorkspaceRatio
from niryo_robot_poses_handlers.srv import GetWorkspaceRobotPoses, GetWorkspaceMatrixPoses, GetWorkspacePoints
from niryo_robot_poses_handlers.srv import GetDynamicFrame, ManageWorkspace
from niryo_robot_poses_handlers.srv import GetPose, ManagePose
from niryo_robot_poses_handlers.srv import ManageDynamicFrame

from niryo_robot_utils.dataclasses.PoseMetadata import PoseMetadata
from niryo_robot_utils.dataclasses.enums import TcpVersion
from niryo_robot_utils.dataclasses.JointsPosition import JointsPosition
from niryo_robot_utils.dataclasses.Pose import Pose


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
        rospy.Subscriber('/niryo_robot_tools_commander/current_id', Int32, self.__callback_tool_id)

        # Grips
        tool_config_dict = rospy.get_param("niryo_robot_tools_commander/tool_list", dict())
        self.__tool_id_gripname_dict = {
            tool["id"]: "default_" + tool["name"].replace(" ", "_")
            for tool in tool_config_dict
        }
        self.__tool_id_gripname_dict[-1] = "default_Calibration_Tip"

        grip_dir = rospy.get_param("~grip_dir")
        self.__grip_manager = GripManager(grip_dir, self.__tool_id_gripname_dict.values())
        rospy.Service('~get_target_pose', GetTargetPose, self.__callback_target_pose)
        rospy.Service('~get_target_pose_v2', GetTargetPose, self.__callback_target_pose_v2)

        # Transform Handlers
        self.__transform_handler = PosesTransformHandler(self.__grip_manager, self)

        rospy.logdebug("Poses Handlers - Transform Handler created")

        # -- POSES
        # Poses
        poses_dir = rospy.get_param("~poses_dir")
        self.__pos_manager = PoseManager(poses_dir)

        rospy.Service('~manage_pose', ManagePose, self.__callback_manage_pose)
        rospy.Service('~get_pose', GetPose, self.__callback_get_pose)
        rospy.Service('~get_pose_list', GetNameDescriptionList, self.__callback_get_pose_list)
        self.__pose_list_publisher = rospy.Publisher('~pose_list', BasicObjectArray, latch=True, queue_size=10)
        self.__publish_pose_list()

        # Dynamic Frame
        self.__tf_listener = None
        self.__tf_buffer = None
        dynamic_frame_dir = rospy.get_param("~dynamic_frame_dir")
        self.dynamic_frame_manager = DynamicFrameManager(dynamic_frame_dir)
        rospy.Service('~manage_dynamic_frame', ManageDynamicFrame, self.__callback_manage_dynamic_frame)
        rospy.Service('~get_dynamic_frame', GetDynamicFrame, self.__callback_get_dynamic_frame)
        rospy.Service('~get_transform_pose', GetTransformPose, self.__callback_get_transform_pose)
        rospy.Service('~get_dynamic_frame_list', GetNameDescriptionList, self.__callback_get_dynamic_frame_list)
        self.__dynamic_frame_list_publisher = rospy.Publisher('~dynamic_frame_list',
                                                              BasicObjectArray,
                                                              queue_size=10,
                                                              latch=True)
        self.__publish_dynamic_frame_list()

        self.dynamic_frame_manager.restore_publisher()
        self.dynamic_frame_manager.publish_frames()

        # Relative pose
        self.__relative_pose_publisher = rospy.Publisher('~relative_pose', PoseStamped, queue_size=1)
        self.__relative_pose_v2_publisher = rospy.Publisher('~relative_pose_v2', PoseStamped, queue_size=1)
        rospy.Subscriber('/niryo_robot/robot_state_v2', RobotState, self.__robot_state_callback, queue_size=1)
        self.__current_pose = [0] * 6

        self.__relative_transform = None
        rospy.Service('~set_relative_transform_frame', SetString, self.__callback_set_relative_transform_frame)
        rospy.Timer(rospy.Duration.from_sec(1 / 30), lambda _: self.__publish_relative_pose())

        # Workspaces
        ws_dir = rospy.get_param("~workspace_dir")

        self.__ws_manager = WorkspaceManager(ws_dir)
        rospy.Service('~manage_workspace', ManageWorkspace, self.__callback_manage_workspace)
        rospy.Service('~get_workspace_ratio', GetWorkspaceRatio, self.__callback_workspace_ratio)
        rospy.Service('~get_workspace_poses', GetWorkspaceRobotPoses, self.__callback_get_workspace_poses)
        rospy.Service('~get_workspace_points', GetWorkspacePoints, self.__callback_get_workspace_points)
        rospy.Service('~get_workspace_matrix_poses', GetWorkspaceMatrixPoses, self.__callback_get_workspace_matrix)
        rospy.Service('~get_workspace_list', GetNameDescriptionList, self.__callback_get_workspace_list)
        self.__workspace_list_publisher = rospy.Publisher('~workspace_list',
                                                          BasicObjectArray,
                                                          latch=True,
                                                          queue_size=10)
        self.__publish_workspace_list()

        if rospy.has_param('~gazebo_workspaces'):
            for ws_name, ws_poses in rospy.get_param('~gazebo_workspaces').items():
                if ws_name not in self.get_available_workspaces()[0]:
                    rospy.loginfo("Poses Handler - Adding the {} workspace...".format(ws_name))
                    self.create_workspace_from_points(ws_name, "", [Point(*point) for point in ws_poses])

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
        if len(workspace.name) > 30:
            rospy.logwarn('Poses Handlers - Workspace name is too long, using : %s instead', workspace.name[:30])
        workspace.name = workspace.name[:30]
        if cmd == req.SAVE:
            if len(workspace.poses) != 4:
                return CommandStatus.WORKSPACE_CREATION_FAILED, "Workspaces have 4 positions, {} given".format(
                    len(workspace.poses))
            try:
                self.create_workspace(workspace.name, workspace.description, workspace.poses)
                self.__publish_workspace_list()
                return CommandStatus.SUCCESS, "Created workspace '{}'".format(workspace.name)
            except NiryoRobotFileException as e:
                return CommandStatus.FILE_ALREADY_EXISTS, str(e)
            except Exception as e:
                return CommandStatus.WORKSPACE_CREATION_FAILED, str(e)

        elif cmd == req.SAVE_WITH_POINTS:
            if len(workspace.points) != 4:
                return CommandStatus.WORKSPACE_CREATION_FAILED, "Workspaces have 4 points, {} given".format(
                    len(workspace.points))
            try:
                self.create_workspace_from_points(workspace.name, workspace.description, workspace.points)
                self.__publish_workspace_list()
                return CommandStatus.SUCCESS, "Created workspace '{}'".format(workspace.name)
            except NiryoRobotFileException as e:
                return CommandStatus.FILE_ALREADY_EXISTS, str(e)
            except Exception as e:
                return CommandStatus.WORKSPACE_CREATION_FAILED, str(e)
        elif cmd == req.DELETE:
            try:
                self.remove_workspace(workspace.name)
                self.__publish_workspace_list()
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

    def __callback_get_workspace_poses(self, req):
        try:
            poses = self.get_workspace_poses(req.name)
            return CommandStatus.SUCCESS, "Success", poses
        except Exception as e:
            return CommandStatus.POSES_HANDLER_READ_FAILURE, str(e), 4 * [RobotState()]

    def __callback_get_workspace_points(self, req):
        try:
            points = self.get_workspace_points(req.name)
            return CommandStatus.SUCCESS, "Success", points
        except Exception as e:
            return CommandStatus.POSES_HANDLER_READ_FAILURE, str(e), 4 * [Point()]

    def __callback_get_workspace_matrix(self, req):
        try:
            position_matrix, orientation_matrix = self.get_workspace_matrix_poses(req.name)
            return CommandStatus.SUCCESS, "Success", position_matrix, orientation_matrix
        except Exception as e:
            return CommandStatus.POSES_HANDLER_READ_FAILURE, str(e), [], []

    def __callback_get_workspace_list(self, _):
        return self.name_description_list_from_fun(self.get_available_workspaces)

    def __publish_workspace_list(self):
        workspace_array = BasicObjectArray()
        workspace_array.objects = [
            BasicObject(name=name, description=description) for name,
            description in zip(*self.get_available_workspaces())
        ]
        self.__workspace_list_publisher.publish(workspace_array)

    # Grips
    def __callback_target_pose(self, req):
        try:
            pose = self.get_target_pose(req.workspace, req.height_offset, req.x_rel, req.y_rel, req.yaw_rel)
            pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw = convert_dh_convention_to_legacy_rpy(pose.rpy.roll,
                                                                                              pose.rpy.pitch,
                                                                                              pose.rpy.yaw)
            return CommandStatus.SUCCESS, "Success", pose
        except Exception as e:
            return CommandStatus.POSES_HANDLER_COMPUTE_FAILURE, str(e), RobotState()

    def __callback_target_pose_v2(self, req):
        try:
            pose = self.get_target_pose(req.workspace, req.height_offset, req.x_rel, req.y_rel, req.yaw_rel)
            return CommandStatus.SUCCESS, "Success", pose
        except Exception as e:
            return CommandStatus.POSES_HANDLER_COMPUTE_FAILURE, str(e), RobotState()

    # Position

    def __callback_get_pose(self, req):
        try:
            pos_msg = self.get_pose(req.name)
            return CommandStatus.SUCCESS, "Success", pos_msg
        except Exception as e:
            return CommandStatus.POSES_HANDLER_READ_FAILURE, str(e), NiryoPose()

    def __callback_get_pose_list(self, _):
        return self.name_description_list_from_fun(self.get_available_poses)

    def __callback_set_relative_transform_frame(self, req):
        if req.value == '':
            self.__relative_transform = None
            return CommandStatus.SUCCESS, 'Disabled the relative transform'
        elif req.value not in self.dynamic_frame_manager.get_all_names():
            return CommandStatus.DYNAMIC_FRAME_DOES_NOT_EXISTS, f'Dynamic frame "{req.value}" does not exists'

        try:
            self.__relative_transform = self.get_transform(req.value, 'world')
        except tf2_ros.LookupException as lookup_exception:
            return CommandStatus.TF_ERROR, str(lookup_exception)
        return CommandStatus.SUCCESS, f'Successfully set relative transform frame to "{req.value}"'

    def __callback_manage_pose(self, req):
        cmd = req.cmd
        pose = req.pose
        if cmd == req.SAVE:
            try:
                tcp_version = 'LEGACY'
                if pose.tcp_version in ['LEGACY', 'DH_CONVENTION']:
                    tcp_version = pose.tcp_version
                elif pose.tcp_version == '':
                    tcp_version = 'LEGACY'
                else:
                    return CommandStatus.POSES_HANDLER_CREATION_FAILED, 'tcp_version is not correct'

                pose_version = pose.pose_version

                pose_obj = PoseObj(name=pose.name,
                                   description=pose.description,
                                   joints=JointsPosition(*pose.joints),
                                   pose=Pose(pose.position.x,
                                             pose.position.y,
                                             pose.position.z,
                                             pose.rpy.roll,
                                             pose.rpy.pitch,
                                             pose.rpy.yaw,
                                             metadata=PoseMetadata(pose_version, TcpVersion[tcp_version])))
                self.__pos_manager.create(pose_obj)
                self.__publish_pose_list()
                return CommandStatus.SUCCESS, "Created Position '{}'".format(pose.name)
            except Exception as e:
                return CommandStatus.POSES_HANDLER_CREATION_FAILED, str(e)
        elif cmd == req.DELETE:
            try:
                self.remove_pose(pose.name)
                self.__publish_pose_list()
                return CommandStatus.SUCCESS, "Removed Position '{}'".format(pose.name)
            except Exception as e:
                return CommandStatus.POSES_HANDLER_REMOVAL_FAILED, str(e)
        else:
            return CommandStatus.UNKNOWN_COMMAND, "cmd '{}' not found.".format(cmd)

    # Dynamic Frame
    def __callback_manage_dynamic_frame(self, req):
        cmd = req.cmd
        frame = req.dynamic_frame
        frame.name = frame.name[:30]
        if cmd == req.SAVE:
            if len(frame.poses) != 3:
                return CommandStatus.DYNAMIC_FRAME_CREATION_FAILED, "Frame have 3 points, {} given".format(
                    len(frame.poses))
            try:
                self.create_dynamic_frame_from_poses(frame.name,
                                                     frame.description,
                                                     frame.poses,
                                                     frame.belong_to_workspace)
                self.__publish_dynamic_frame_list()
                return CommandStatus.SUCCESS, "Created dynamic frame '{}'".format(frame.name)
            except NiryoRobotFileException as e:
                return CommandStatus.FILE_ALREADY_EXISTS, str(e)
            except (ValueError, Exception) as e:
                return CommandStatus.POSES_HANDLER_CREATION_FAILED, str(e)
        if cmd == req.SAVE_WITH_POINTS:
            if len(frame.points) != 3:
                return CommandStatus.DYNAMIC_FRAME_CREATION_FAILED, "Frame have 3 points, {} given".format(
                    len(frame.points))
            try:
                self.create_dynamic_frame_from_points(frame.name,
                                                      frame.description,
                                                      frame.points,
                                                      frame.belong_to_workspace)
                self.__publish_dynamic_frame_list()
                return CommandStatus.SUCCESS, "Created dynamic frame '{}'".format(frame.name)
            except NiryoRobotFileException as e:
                return CommandStatus.FILE_ALREADY_EXISTS, str(e)
            except (ValueError, Exception) as e:
                return CommandStatus.POSES_HANDLER_CREATION_FAILED, str(e)
        elif cmd == req.DELETE:
            if frame.belong_to_workspace:
                return (CommandStatus.POSES_HANDLER_REMOVAL_FAILED,
                        "Can't remove a dynamic frame which belong to a workspace")
            try:
                self.remove_dynamic_frame(frame.name, frame.belong_to_workspace)
                self.__publish_dynamic_frame_list()
                return CommandStatus.SUCCESS, "Removed dynamic frame '{}'".format(frame.name)
            except Exception as e:
                return CommandStatus.POSES_HANDLER_REMOVAL_FAILED, str(e)
        elif cmd == req.EDIT:
            if frame.belong_to_workspace:
                return CommandStatus.DYNAMIC_FRAME_EDIT_FAILED, "Can't edit a dynamic frame which belong to a workspace"
            try:
                if frame.description != '':
                    self.dynamic_frame_manager.edit_description(frame.name, frame.description)
                if len(frame.points) == 3:
                    points_as_list = [(p.x, p.y, p.z) for p in frame.points]
                    self.dynamic_frame_manager.edit_points(frame.name, points_as_list)
                if frame.rpy != RPY():
                    self.dynamic_frame_manager.edit_static_transform_w_rpy(frame.name, frame.position, frame.rpy)
                if frame.orientation != Quaternion():
                    self.dynamic_frame_manager.edit_static_transform(frame.name, frame.position, frame.orientation)
                if frame.new_name != '':
                    self.dynamic_frame_manager.edit_name(frame.name, frame.new_name)

                self.__publish_dynamic_frame_list()
                return CommandStatus.SUCCESS, "Edited dynamic frame '{}'".format(frame.name)
            except Exception as e:
                return CommandStatus.DYNAMIC_FRAME_EDIT_FAILED, str(e)
        else:
            return CommandStatus.UNKNOWN_COMMAND, "cmd '{}' not found.".format(cmd)

    def __callback_get_dynamic_frame(self, req):
        try:
            dyn_frame = self.get_dynamic_frame(req.name)
            return CommandStatus.SUCCESS, "Success", dyn_frame
        except Exception as e:
            return CommandStatus.POSES_HANDLER_READ_FAILURE, str(e), None

    def __callback_get_transform_pose(self, req):
        try:
            pose = [req.position.x, req.position.y, req.position.z, req.rpy.roll, req.rpy.pitch, req.rpy.yaw]
            transform_pose = self.transform_pose_from_frame(pose, req.source_frame, req.local_frame)
            position = transform_pose.pose.position
            rpy = self.quaternion_to_rpy(transform_pose.pose.orientation)
            return CommandStatus.SUCCESS, "Success", position, rpy

        except Exception as e:
            rospy.logerr("Poses Handlers - Error occurred when getting transform: {}".format(e))
            return CommandStatus.CONVERT_FAILED, str(e), Point(), RPY()

    def __callback_get_dynamic_frame_list(self, _):
        return self.name_description_list_from_fun(self.get_available_dynamic_frame_w_description)

    def __publish_dynamic_frame_list(self):
        dynamic_frame_array = BasicObjectArray()
        dynamic_frame_array.objects = [
            BasicObject(name=name, description=description) for name,
            description in zip(*self.get_available_dynamic_frame_w_description())
        ]
        self.__dynamic_frame_list_publisher.publish(dynamic_frame_array)

    def __publish_pose_list(self):
        pose_array = BasicObjectArray()
        pose_array.objects = [
            BasicObject(name=name, description=description) for name, description in zip(*self.get_available_poses())
        ]
        self.__pose_list_publisher.publish(pose_array)

    def __robot_state_callback(self, msg):
        self.__current_pose = [msg.position.x, msg.position.y, msg.position.z, msg.rpy.roll, msg.rpy.pitch, msg.rpy.yaw]

    def __publish_relative_pose(self):
        if self.__relative_transform is None:
            return

        transform_pose_v2 = self.transform_pose(self.__relative_transform, self.__current_pose)
        self.__relative_pose_v2_publisher.publish(transform_pose_v2)

        current_pose_v1 = self.__current_pose[:3] + list(convert_dh_convention_to_legacy_rpy(*self.__current_pose[3:]))
        transform_pose_v1 = self.transform_pose(self.__relative_transform, current_pose_v1)
        self.__relative_pose_publisher.publish(transform_pose_v1)

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
            pose_obj = [[pose.position.x, pose.position.y, pose.position.z],
                        [pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw]]
            robot_poses_raw.append(pose_obj)

        self.__ws_manager.create(name, points, robot_poses_raw, description)

        # Asssociate frame
        self.dynamic_frame_manager.create(name, [points[0]] + [points[3]] + [points[1]], description, True)

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

        # Asssociate frame
        self.dynamic_frame_manager.create(name, [points_raw[0]] + [points_raw[3]] + [points_raw[1]], description, True)

    def remove_workspace(self, name):
        """
        Removes a workspace
        :param name: name of the workspace to remove
        """
        self.__ws_manager.remove(name)

        self.dynamic_frame_manager.remove(name, True)

    def get_available_workspaces(self):
        """
        Returns a list of all available workspace names.
        """
        return self.__ws_manager.get_all_names_w_description()

    def get_workspace_poses(self, name):
        ws_raw = self.__ws_manager.read(name)
        poses = [RobotState(position=Point(*pose[0]), rpy=RPY(*pose[1])) for pose in ws_raw.robot_poses]
        return poses

    def get_workspace_points(self, name):
        ws_raw = self.__ws_manager.read(name)
        points = [Point(*pose) for pose in ws_raw.points]
        return points

    def get_workspace_matrix_poses(self, name):
        """
        Returns position and orientation matrix of a workspace
        """
        ws_raw = self.__ws_manager.read(name)

        position_raw = ws_raw.position_matrix.tolist()
        orientation_raw = ws_raw.rotation_matrix.tolist()

        position_matrix = [Point(*pose) for pose in position_raw]
        orientation_matrix = [Quaternion(*pose) for pose in orientation_raw]
        return position_matrix, orientation_matrix

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
        self.__transform_handler.set_relative_pose_object(current_ws,
                                                          x_rel,
                                                          y_rel,
                                                          yaw_rel,
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

    def get_pose(self, name):
        """
        Get pose from pose manager and return NiryoPose

        :param name: pose name
        :type name: str
        :return: The pose object
        :rtype: NiryoPose
        """
        pose_obj = self.__pos_manager.read(name)
        pose = NiryoPose()
        pose.name = pose_obj.name
        pose.description = pose_obj.description
        pose.joints = list(pose_obj.joints)
        pose.position = Point(x=pose_obj.pose.x, y=pose_obj.pose.y, z=pose_obj.pose.z)
        pose.rpy = RPY(roll=pose_obj.pose.roll, pitch=pose_obj.pose.pitch, yaw=pose_obj.pose.yaw)
        rx, ry, rz, w = pose_obj.pose.quaternion()
        pose.orientation = Quaternion(x=rx, y=ry, z=rz, w=w)
        try:
            pose.pose_version = pose_obj.pose.metadata.version
        except Exception:
            # pose_version doesn't exist assume it has legacy format
            pose.pose_version = 0
        try:
            pose.tcp_version = pose_obj.pose.metadata.tcp_version.name
        except Exception:
            # tcp_version doesn't exist assume it has legacy format
            pose.tcp_version = 'LEGACY'
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

    # Dynamic frame
    def create_dynamic_frame_from_poses(self, name, description, poses, belong_to_workspace=False):
        """
        Create a dynamic frame from ManageDynamicFrame message fields

        :param name: nom de la frame
        :type name: str
        :param description : description
        :type description: str
        :param poses: list of poses to create the frame
        :type poses: niryo_robot_msgs/RobotState
        :return: None
        """
        points = []

        for pose in poses:
            rospy.logdebug("Robot point\n{}".format(pose.position))
            point = self.__transform_handler.get_calibration_tip_position(pose)
            rospy.logdebug("Tip point\n{}".format(point))
            points.append([point.x, point.y, point.z])

        self.dynamic_frame_manager.create(name, points, description, belong_to_workspace)

    def create_dynamic_frame_from_points(self, name, description, points, belong_to_workspace=False):
        """
        Create a dynamic frame from ManageDynamicFrame message fields

        :param name: nom de la frame
        :type name: str
        :param description : description
        :type description: str
        :param points: list of points to create the frame
        :type points: geometry_msg/Point
        :return: None
        """
        points_raw = []
        for point in points:
            points_raw.append([point.x, point.y, point.z])

        self.dynamic_frame_manager.create(name, points_raw, description, belong_to_workspace)

    def remove_dynamic_frame(self, name, belong_to_workspace=False):
        """
        Asks dynamic frame manager to remove dynamic frame

        :param name: dynamic frame name
        :type name: str
        :param belong_to_workspace: whether the dynamic frame belong to a workspace or not
        :type belong_to_workspace: bool
        :return: None
        """
        self.dynamic_frame_manager.remove(name, belong_to_workspace)

    def get_dynamic_frame(self, name):
        """
        Get dynamic frame from dynamic frame manager and return pose of the frame

        :param name: pose name
        :type name: str
        :return: Pose of the dynamic frame object
        :rtype: Pose
        """
        frame_read = self.dynamic_frame_manager.read(name)
        pose_obj = frame_read.static_transform_stamped

        frame = DynamicFrame()
        frame.name = frame_read.name
        frame.description = frame_read.description
        frame.belong_to_workspace = frame_read.belong_to_workspace
        point = Point(*pose_obj[0])
        quaternion = Quaternion(*pose_obj[1])
        roll, pitch, yaw = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        rpy = RPY(roll, pitch, yaw)

        frame.position = point
        frame.orientation = quaternion
        frame.rpy = rpy

        return frame

    def get_available_dynamic_frame_w_description(self):
        """
        Ask the dynamic frame manager which dynamic frame are available

        :return: list of dynamic frame name
        :rtype: list[str]
        """
        return self.dynamic_frame_manager.get_all_names_w_description()

    def get_transform(self, source_frame, local_frame):
        if self.__tf_buffer is None or self.__tf_listener is None:
            self.__tf_buffer = tf2_ros.Buffer()
            self.__tf_listener = tf2_ros.TransformListener(self.__tf_buffer)

        # Get transform
        transform = self.__tf_buffer.lookup_transform(source_frame, local_frame, rospy.Time(0), rospy.Duration(4))
        return transform

    def transform_pose(self, transform, pose):
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose.position = Point(*pose[:3])
        quaternion = tf.transformations.quaternion_from_euler(*pose[3:])
        pose_stamped.pose.orientation = Quaternion(*quaternion)
        pose_stamped.header.frame_id = transform.child_frame_id
        pose_stamped.header.stamp = rospy.Time.now()

        # Calculate pose in world frame
        transform_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

        return transform_pose

    def transform_pose_from_frame(self, pose, source_frame, local_frame):
        """
        Get the transform pose in source_frame from local_frame

        :return: transform pose
        :rtype : list[float]
        """

        transform = self.get_transform(source_frame, local_frame)
        return self.transform_pose(transform, pose)

    @staticmethod
    def quaternion_to_rpy(quaternion):
        euler = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        return RPY(*euler)

    @staticmethod
    def name_description_list_from_fun(fun):
        response = GetNameDescriptionListResponse()
        response.status = CommandStatus.SUCCESS

        try:
            response.name_list, response.description_list = fun()
        except Exception as e:
            rospy.logerr("Poses Handlers - Error occured when getting list: {}".format(e))
            response.status = CommandStatus.POSES_HANDLER_READ_FAILURE
            response.message = str(e)

        response.objects = [
            BasicObject(name=name, description=description) for name,
            description in zip(response.name_list, response.description_list)
        ]
        return response


if __name__ == "__main__":
    sentry_init()

    rospy.init_node('niryo_robot_poses_handlers', anonymous=False, log_level=rospy.INFO)

    # change logger level according to node parameter
    log_level = rospy.get_param("~log_level")
    logger = logging.getLogger("rosout")
    logger.setLevel(log_level)

    try:
        vision_node = PoseHandlerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
