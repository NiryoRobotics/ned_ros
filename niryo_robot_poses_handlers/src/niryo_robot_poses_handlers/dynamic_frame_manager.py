#!/usr/bin/env python

from niryo_robot_poses_handlers.file_manager import FileManager, NiryoRobotFileException
from niryo_robot_poses_handlers.transform_functions import euler_from_matrix, quaternion_from_euler, normalize_quaterion

from geometry_msgs.msg import Point, Vector3, TransformStamped, Quaternion

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
        self.static_transform_stamped = []

    def to_dict(self):
        dict_ = dict()
        dict_["name"] = self.name
        dict_["description"] = self.description
        dict_["belong_to_workspace"] = self.belong_to_workspace
        dict_["static_transform_stamped"] = self.static_transform_stamped
        return dict_

    @classmethod
    def from_dict(cls, dict_):
        dyn_frame = cls(dict_["name"], dict_["description"])
        dyn_frame.belong_to_workspace = dict_["belong_to_workspace"]
        dyn_frame.static_transform_stamped = dict_["static_transform_stamped"]

        return dyn_frame

    def get_value(self):
        return self.static_transform_stamped, self.robot_poses


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
    def __get_static_transform(name, points):
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

        # Create frame
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.frame_id = "world"
        static_transform_stamped.child_frame_id = name
        # Position
        static_transform_stamped.transform.translation = Vector3(*[point_o.x, point_o.y, point_o.z])
        # Orientation
        static_transform_stamped.transform.rotation = Quaternion(*q)

        return static_transform_stamped, [[point_o.x, point_o.y, point_o.z], q.tolist()]

    def create(self, name, points, description="", belong_to_workspace=False):
        """
        Create a new local frame
        """
        static_transform_stamped, transform_as_list = self.__get_static_transform(name, points)

        # Add to the list of publish transform
        self.dict_dynamic_frame[name] = {"transform": static_transform_stamped}

        dynamic_frame = DynamicFrame(name, description)
        dynamic_frame.points = points
        dynamic_frame.belong_to_workspace = belong_to_workspace
        dynamic_frame.static_transform_stamped = transform_as_list

        self._write(name, dynamic_frame)

    def edit_name(self, name, new_name):
        self.check_exist(name)

        transform_stamped = self.dict_dynamic_frame.pop(name)
        transform_stamped['transform'].child_frame_id = new_name
        self.dict_dynamic_frame[new_name] = transform_stamped

        dynamic_frame = self.read(name)
        FileManager.remove(self, name)
        dynamic_frame.name = new_name
        self._write(new_name, dynamic_frame)

    def edit_description(self, name, description):
        self.check_exist(name)

        dynamic_frame = self.read(name)
        dynamic_frame.description = description
        self._write(name, dynamic_frame)

    def edit_points(self, name, points):
        self.check_exist(name)

        static_transform_stamped, transform_as_list = self.__get_static_transform(name, points)

        # Add to the list of publish transform
        self.dict_dynamic_frame[name] = {"transform": static_transform_stamped}

        dynamic_frame = self.read(name)
        dynamic_frame.points = points
        dynamic_frame.static_transform_stamped = transform_as_list
        self._write(name, dynamic_frame)

    def edit_static_transform_w_rpy(self, name, position, rpy):
        quaternion = Quaternion(*quaternion_from_euler(rpy.roll, rpy.pitch, rpy.yaw))
        return self.edit_static_transform(name, position, quaternion)

    def edit_static_transform(self, name, position, quaternion):
        self.check_exist(name)

        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.frame_id = "world"
        static_transform_stamped.child_frame_id = name
        # Position
        static_transform_stamped.transform.translation = Vector3(*[position.x, position.y, position.z])

        # Orientation
        normalized_quaterion = normalize_quaterion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        static_transform_stamped.transform.rotation = Quaternion(*normalized_quaterion)

        # Add to the list of publish transform
        self.dict_dynamic_frame[name] = {"transform": static_transform_stamped}

        dynamic_frame = self.read(name)
        # we set everything to 0 as we are not able to calculate the points from the transform
        dynamic_frame.points = [[0] * 3] * 3
        dynamic_frame.static_transform_stamped = [[position.x, position.y, position.z],
                                                  [quaternion.x, quaternion.y, quaternion.z, quaternion.z]]
        self._write(name, dynamic_frame)

    def remove(self, name, belong_to_workspace=False):
        """
        Remove a frame
        """
        if not belong_to_workspace:
            frame = self.read(name)
            if frame.belong_to_workspace:
                raise NiryoRobotFileException("Frame {} can't be removed because it belong to a workspace".format(name))

        self.check_exist(name)

        # Remove transform from publish transform list
        self.dict_dynamic_frame.pop(name)
        FileManager.remove(self, name)

    def restore_publisher(self):
        """
        Publish frame present in files
        """
        list_files = self.get_all_names()
        for file in list_files:
            frame = self.read(file)

            # Create frame
            static_transform_stamped = TransformStamped()
            static_transform_stamped.header.frame_id = "world"
            static_transform_stamped.child_frame_id = frame.name
            # Position
            static_transform_stamped.transform.translation = Vector3(*frame.static_transform_stamped[0])
            # Orientation
            static_transform_stamped.transform.rotation = Quaternion(*frame.static_transform_stamped[1])

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
            try:
                self.__broadcaster.sendTransform(frame["transform"])
            except Exception as e:
                rospy.logerr(frame)
                raise e

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
