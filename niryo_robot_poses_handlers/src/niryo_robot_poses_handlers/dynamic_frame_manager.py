#!/usr/bin/env python

from asyncore import read
from niryo_robot_poses_handlers.file_manager import FileManager, NiryoRobotFileException
from niryo_robot_poses_handlers.transform_functions import euler_from_matrix, quaternion_from_euler

from geometry_msgs.msg import Point, Vector3, TransformStamped, Pose, Quaternion

import tf2_ros
import numpy as np
import rospy


class DynamicFrame(object):
    """
    """

    def __init__(self, name, description):
        self.name = name
        self.description = description
        self.belong_to_workspace = False
        self.robot_poses = []
        self.points = []
        self.static_transform_stamped = []

    def to_dict(self):
        dict_ = dict()
        dict_["name"] = self.name
        dict_["description"] = self.description
        dict_["belong_to_workspace"] = self.belong_to_workspace
        dict_["points"] = self.points
        dict_["static_transform_stamped"] = self.static_transform_stamped
        return dict_

    @classmethod
    def from_dict(cls, dict_):
        dyn_frame = cls(dict_["name"], dict_["description"])
        dyn_frame.points = dict_["points"]
        dyn_frame.belong_to_workspace = dict_["belong_to_workspace"]
        dyn_frame.static_transform_stamped = dict_["static_transform_stamped"]

        return dyn_frame

    def get_value(self):
        return self.static_transform_stamped, self.points, self.robot_poses


class DynamicFrameManager(FileManager):
    """
    Manages the creation, storage and loading of dynamic frame.

    :raises NiryoRobotFileException:
    """
    object_type = DynamicFrame
    dict_dynamic_frame = {}

    def __init__(self, ws_dir):
        FileManager.__init__(self, ws_dir, "dynamic_frame")

        self.__publish_timer = None
        self.__broadcaster = tf2_ros.TransformBroadcaster()

    @staticmethod
    def calculate_transform(points):
        """
        Calculate frame with 3 poses
        """
        # Points
        point_o = Point(*points[0])
        point_vx = Point(*points[1])
        point_vy = Point(*points[2])

        # Acquisition vector
        vector_x = np.array([point_vx.x - point_o.x, point_vx.y - point_o.y, point_vx.z - point_o.z])
        vector_y = np.array([point_vy.x - point_o.x, point_vy.y - point_o.y, point_vy.z - point_o.z])

        # Calculate normal vector vz
        vector_z = np.cross(vector_x, vector_y)
        if np.all(vector_z == 0):
            raise NiryoRobotFileException("Frame can't be created because 3 points are aligned")
        # Recalculate vector vy
        vector_y = np.cross(vector_z, vector_x)

        # Normalize
        vector_x = vector_x / np.linalg.norm(vector_x)
        vector_y = vector_y / np.linalg.norm(vector_y)
        vector_z = vector_z / np.linalg.norm(vector_z)

        # Calculate rotation matrix
        rotation = np.eye(4)
        rotation[:3, :3] = np.stack([vector_x, vector_y, vector_z]).T

        # Get quaternion
        al, be, ga = euler_from_matrix(rotation)
        q = quaternion_from_euler(al, be, ga)

        return point_o, point_vx, point_vy, q

    def create(self, frame_name, points, description="", belong_to_workspace=False):
        """
        Create a new local frame
        """
        print(points)
        if self.exists(frame_name):
            raise NiryoRobotFileException("Frame {} already exists".format(frame_name))

        point_o, point_vx, point_vy, q = self.calculate_transform(points)

        # Create frame
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.frame_id = "world"
        static_transform_stamped.child_frame_id = frame_name
        # Position
        static_transform_stamped.transform.translation = Vector3(*[point_o.x, point_o.y, point_o.z])
        # Orientation
        static_transform_stamped.transform.rotation = Quaternion(*q)

        # Add to the list of publish transform
        self.dict_dynamic_frame[frame_name] = {"transform": static_transform_stamped}

        dynamic_frame = DynamicFrame(frame_name, description)
        dynamic_frame.points = points
        dynamic_frame.belong_to_workspace = belong_to_workspace
        dynamic_frame.static_transform_stamped = [[point_o.x, point_o.y, point_o.z], q.tolist()]

        self._write(frame_name, dynamic_frame)

    def edit_frame(self, frame_name, new_frame_name, description=""):
        """
            Edit a frame
        """
        if not self.exists(frame_name):
            raise NiryoRobotFileException("Frame {} not already exists".format(frame_name))

        # Remove transform from publish transform list
        self.dict_dynamic_frame.pop(frame_name)

        dynamic_frame = self.read(frame_name)
        points = dynamic_frame.points

        point_o, point_vx, point_vy, q = self.calculate_transform(points)

        # Create frame
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.frame_id = "world"
        static_transform_stamped.child_frame_id = new_frame_name
        # Position
        static_transform_stamped.transform.translation = Vector3(*[point_o.x, point_o.y, point_o.z])
        # Orientation
        static_transform_stamped.transform.rotation = Quaternion(*q)

        # Add to the list of publish transform
        self.dict_dynamic_frame[new_frame_name] = {"transform": static_transform_stamped}

        dynamic_frame = DynamicFrame(new_frame_name, description)
        dynamic_frame.points = points
        dynamic_frame.belong_to_workspace = dynamic_frame.belong_to_workspace
        dynamic_frame.static_transform_stamped = [[point_o.x, point_o.y, point_o.z], q.tolist()]

        self._write(new_frame_name, dynamic_frame)

        # Case : edit only description
        if (frame_name != new_frame_name):
            FileManager.remove(self, frame_name)

    def remove(self, frame_name, belong_to_workspace=False):
        """
        Remove a frame
        """
        if not belong_to_workspace:
            frame = self.read(frame_name)
            if frame.belong_to_workspace:
                raise NiryoRobotFileException("Frame {} can't be removed because it belong to a workspace".format(
                    frame_name))

        if not self.exists(frame_name):
            raise NiryoRobotFileException("Frame {} not already exists".format(frame_name))

        # Remove transform from publish transform list
        self.dict_dynamic_frame.pop(frame_name)
        FileManager.remove(self, frame_name)

    def restore_publisher(self):
        """
        Publish frame present in files
        """
        list_files = self.get_all_names()
        for file in list_files:
            frame = self.read(file)
            points_raw = []
            for i in range(3):
                points_raw.append([frame.points[i][0], frame.points[i][1], frame.points[i][2]])

            point_o, point_vx, point_vy, q = self.calculate_transform(points_raw)

            # Create frame
            static_transform_stamped = TransformStamped()
            static_transform_stamped.header.frame_id = "world"
            static_transform_stamped.child_frame_id = str(frame.name)
            # Position
            static_transform_stamped.transform.translation = Vector3(*[point_o.x, point_o.y, point_o.z])
            # Orientation
            static_transform_stamped.transform.rotation = Quaternion(*q)

            # Add to the list of publish transform
            self.dict_dynamic_frame[frame.name] = {"transform": static_transform_stamped}

    def callback_publisher(self, _event):
        """
        Callback publisher
        """
        current_time = rospy.Time.now()
        for frame in list(self.dict_dynamic_frame.values()):
            # Actualisation du temps
            frame["transform"].header.stamp = current_time
            self.__broadcaster.sendTransform(frame["transform"])

    def publish_frames(self):
        """
        Publish frames
        """
        if self.__publish_timer is None:
            self.__publish_timer = rospy.Timer(rospy.Duration(1), self.callback_publisher)

    def stop_publish_frames(self):
        """
        Stop Publishing frames
        """
        if isinstance(self.__publish_timer, rospy.Timer):
            self.__publish_timer.shutdown()
            self.__publish_timer = None
