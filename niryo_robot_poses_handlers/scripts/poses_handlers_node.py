#!/usr/bin/env python

# Libs
import rospy
import logging
import thread

from transform_handler import PosesTransformHandler
from niryo_robot_poses_handlers.transform_functions import euler_from_quaternion
from niryo_robot_poses_handlers.file_manager import NiryoRobotFileException
from niryo_robot_poses_handlers.grip_manager import GripManager
from niryo_robot_poses_handlers.pose_manager import PoseManager
from niryo_robot_poses_handlers.workspace_manager import WorkspaceManager
from niryo_robot_poses_handlers.dynamic_frame_manager import DynamicFrameManager

# Command Status
from niryo_robot_msgs.msg import CommandStatus

# Messages
from geometry_msgs.msg import Point, Quaternion
from niryo_robot_msgs.msg import RobotState
from niryo_robot_msgs.msg import RPY
from niryo_robot_poses_handlers.msg import NiryoPose, DynamicFrame
from std_msgs.msg import Int32
from niryo_robot_tools_commander.msg import TCP

# Services
from niryo_robot_msgs.srv import GetNameDescriptionList

from niryo_robot_poses_handlers.srv import GetTargetPose, GetTransformPose
from niryo_robot_poses_handlers.srv import GetWorkspaceRatio
from niryo_robot_poses_handlers.srv import GetWorkspaceRobotPoses, GetWorkspaceMatrixPoses, GetWorkspacePoints
from niryo_robot_poses_handlers.srv import GetDynamicFrame, ManageWorkspace
from niryo_robot_poses_handlers.srv import GetPose, ManagePose
from niryo_robot_poses_handlers.srv import ManageDynamicFrame


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
        self.__tool_id_gripname_dict = {tool["id"]: "default_" + tool["name"].replace(" ", "_")
                                        for tool in tool_config_dict}
        self.__tool_id_gripname_dict[-1] = "default_Calibration_Tip"

        grip_dir = rospy.get_param("~grip_dir")
        self.__grip_manager = GripManager(grip_dir, self.__tool_id_gripname_dict.values())
        rospy.Service('~get_target_pose', GetTargetPose, self.__callback_target_pose)

        # Transform Handlers
        self.__transform_handler = PosesTransformHandler(self.__grip_manager, self)

        rospy.logdebug("Poses Handlers - Transform Handler created")

        # -- POSES
        # Poses
        poses_dir = rospy.get_param("~poses_dir")
        self.__pos_manager = PoseManager(poses_dir)

        rospy.Service('~manage_pose', ManagePose, self.__callback_manage_pose)
        rospy.Service('~get_pose', GetPose, self.__callback_get_pose)
        rospy.Service('~get_pose_list', GetNameDescriptionList, self.__callback_get_position_list)

        # Dynamic Frame
        self.__tf_listener = None
        self.__tf_buffer = None
        dynamic_frame_dir = rospy.get_param("~dynamic_frame_dir")
        self.dynamic_frame_manager = DynamicFrameManager(dynamic_frame_dir)
        rospy.Service('~manage_dynamic_frame', ManageDynamicFrame, self.__callback_manage_dynamic_frame)
        rospy.Service('~get_dynamic_frame', GetDynamicFrame, self.__callback_get_dynamic_frame)
        rospy.Service('~get_dynamic_frame_list', GetNameDescriptionList, self.__callback_get_dynamic_frame_list)
        rospy.Service('~get_transform_pose', GetTransformPose, self.__callback_get_transform_pose)
        self.dynamic_frame_manager.restore_publisher()
        # Publisher dynamic frames
        thread.start_new_thread(self.dynamic_frame_manager.publish_frames, ())

        # Workspaces
        ws_dir = rospy.get_param("~workspace_dir")

        self.__ws_manager = WorkspaceManager(ws_dir)
        rospy.Service('~manage_workspace', ManageWorkspace, self.__callback_manage_workspace)
        rospy.Service('~get_workspace_ratio', GetWorkspaceRatio, self.__callback_workspace_ratio)
        rospy.Service('~get_workspace_list', GetNameDescriptionList, self.__callback_workspace_list)
        rospy.Service('~get_workspace_poses', GetWorkspaceRobotPoses, self.__callback_get_workspace_poses)
        rospy.Service('~get_workspace_points', GetWorkspacePoints, self.__callback_get_workspace_points)
        rospy.Service('~get_workspace_matrix_poses', GetWorkspaceMatrixPoses, self.__callback_get_workspace_matrix)

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
                self.create_dynamic_frame_from_poses(frame.name, frame.description, frame.poses,
                                                     frame.belong_to_workspace)
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
                self.create_dynamic_frame_from_points(frame.name, frame.description, frame.points,
                                                      frame.belong_to_workspace)
                return CommandStatus.SUCCESS, "Created dynamic frame '{}'".format(frame.name)
            except NiryoRobotFileException as e:
                return CommandStatus.FILE_ALREADY_EXISTS, str(e)
            except (ValueError, Exception) as e:
                return CommandStatus.POSES_HANDLER_CREATION_FAILED, str(e)
        elif cmd == req.DELETE:
            try:
                self.remove_dynamic_frame(frame.name, frame.belong_to_workspace)
                return CommandStatus.SUCCESS, "Removed dynamic frame '{}'".format(frame.name)
            except Exception as e:
                return CommandStatus.POSES_HANDLER_REMOVAL_FAILED, str(e)
        elif cmd == req.EDIT:
            try:
                self.edit_dynamic_frame(frame.name, frame.new_name, frame.description)
                return CommandStatus.SUCCESS, "Edited dynamic frame '{}'".format(frame.name)
            except Exception as e:
                return CommandStatus.DYNAMIC_FRAME_EDIT_FAILED, str(e)
        else:
            return CommandStatus.UNKNOWN_COMMAND, "cmd '{}' not found.".format(frame.cmd)

    def __callback_get_dynamic_frame(self, req):
        try:
            dyn_frame = self.get_dynamic_frame(req.name)
            return CommandStatus.SUCCESS, "Success", dyn_frame
        except Exception as e:
            return CommandStatus.POSES_HANDLER_READ_FAILURE, str(e), None

    def __callback_get_dynamic_frame_list(self, _):
        try:
            dynamic_frame_list, description_list = self.get_available_dynamic_frame_w_description()
        except Exception as e:
            rospy.logerr("Poses Handlers - Error occured when getting dynamic frames list: {}".format(e))
            dynamic_frame_list = description_list = []
        return {"name_list": dynamic_frame_list, "description_list": description_list}

    def __callback_get_transform_pose(self, req):
        try:
            pose = [req.position.x, req.position.y, req.position.z, req.rpy.roll, req.rpy.pitch, req.rpy.yaw]
            position, rpy = self.get_transform_pose(pose, req.source_frame, req.local_frame)
            return CommandStatus.SUCCESS, "Success", position, rpy

        except Exception as e:
            rospy.logerr("Poses Handlers - Error occured when getting transform: {}".format(e))
            position = rpy = []
            return CommandStatus.CONVERT_FAILED, str(e), position, rpy

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
            rospy.loginfo("Robot point\n{}".format(pose.position))
            point = self.__transform_handler.get_calibration_tip_position(pose)
            rospy.loginfo("Tip point\n{}".format(point))
            points.append([point.x, point.y, point.z])
            pose_raw = [[pose.position.x, pose.position.y, pose.position.z],
                        [pose.rpy.roll, pose.rpy.pitch, pose.rpy.yaw]]
            robot_poses_raw.append(pose_raw)

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
        poses = [RobotState(position=Point(*pose[0]), rpy=RPY(*pose[1]))
                 for pose in ws_raw.robot_poses]
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
            rospy.loginfo("Robot point\n{}".format(pose.position))
            point = self.__transform_handler.get_calibration_tip_position(pose)
            rospy.loginfo("Tip point\n{}".format(point))
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
        :return: None
        """
        self.dynamic_frame_manager.remove(name, belong_to_workspace)

    def edit_dynamic_frame(self, name, new_name, description):
        """
        Asks dynamic frame manager to modify dynamic frame

        :param name: dynamic frame name
        :type name: str
        :param new_name: new dynamic frame name
        :type new_name: str
        :param description: new description
        :type description: str
        :type name: str
        :return: None
        """
        self.dynamic_frame_manager.edit_frame(name, new_name, description)

    def get_dynamic_frame(self, name):
        """
        Get dynamic frame from dynamic frame manager and return pose of the frame

        :param name: pose name
        :type name: str
        :return: Pose of the dynamic frame object
        :rtype: Pose
        """
        frame_read = self.dynamic_frame_manager.read(name)
        pose_raw = frame_read.static_transform_stamped

        frame = DynamicFrame()
        frame.name = frame_read.name
        frame.description = frame_read.description
        point = Point(*pose_raw[0])
        quaternion = Quaternion(*pose_raw[1])
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

    def get_transform_pose(self, pose, source_frame, local_frame):
        """
        Get the transform pose in source_frame from local_frame

        :return: transform pose
        :rtype : list[float]
        """
        import tf2_ros
        import tf2_geometry_msgs
        import tf

        if self.__tf_buffer is None or self.__tf_listener is None:
            self.__tf_buffer = tf2_ros.Buffer()
            self.__tf_listener = tf2_ros.TransformListener(self.__tf_buffer)

        # Get transform
        try:
            transform = self.__tf_buffer.lookup_transform(source_frame, local_frame, rospy.Time(), rospy.Duration(4.0))
        except Exception as e:
            return str(e)

        # Pose in local_frame
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose.position = Point(*pose[:3])
        quaternion = tf.transformations.quaternion_from_euler(*pose[3:])
        pose_stamped.pose.orientation = Quaternion(*quaternion)
        pose_stamped.header.frame_id = local_frame
        pose_stamped.header.stamp = rospy.Time.now()

        # Calculate pose in world frame
        transform_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

        quaternion = transform_pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion(self.quaternion_to_list(quaternion))
        position = transform_pose.pose.position

        rpy = RPY(*euler)

        return position, rpy

    @staticmethod
    def quaternion_to_list(quaternion):
        return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


if __name__ == "__main__":
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
