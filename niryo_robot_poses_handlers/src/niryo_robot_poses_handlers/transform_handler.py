#!/usr/bin/env python

# Libs
import copy

import rospy
import tf2_ros
import niryo_robot_poses_handlers.transform_functions as transformations
import threading
import numpy as np

# Messages
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from visualization_msgs.msg import Marker


class PosesTransformHandler:
    """
    This class uses a tfBuffer to handle transforms related to the vision kit.
    """

    def __init__(self, grip_manager, pose_handler_node):
        self.__tf_buffer = tf2_ros.Buffer()
        self.__debug_stop_event = threading.Event()
        self.__debug_thread = None
        self.__debug_current_ws = None  # only for debugging purposes
        self.__pose_handler_node = pose_handler_node
        self.__grip_manager = grip_manager

    def __del__(self):
        self.disable_debug()

    def set_relative_pose_object(self, workspace, x_rel, y_rel, yaw_rel, yaw_center=None):
        """
        Updates the transform base_link -> object_base in local tfBuffer

        :param workspace: reference workspace object
        :param x_rel: object base x position relative to workspace
        :param y_rel: object base y position relative to workspace
        :param yaw_rel: object base rotation on z relative to workspace
        :param yaw_center: Avoid over rotation
        """
        tmp_buffer = tf2_ros.Buffer()
        stamp = rospy.Time.now()
        delta_x = transformations.euclidian_dist(workspace.points[1], workspace.points[0]) * x_rel
        delta_y = transformations.euclidian_dist(workspace.points[3], workspace.points[0]) * y_rel

        t = TransformStamped()
        t.transform.translation = Vector3(delta_y, delta_x, 0)
        t.transform.rotation = Quaternion(*transformations.quaternion_from_euler(0, 0, -yaw_rel))
        t.header.frame_id = str(workspace.name)
        t.header.stamp = stamp
        t.child_frame_id = "object_base"
        tmp_buffer.set_transform(t, "default_authority")

        transform_frame = self.__pose_handler_node.dynamic_frame_manager.dict_dynamic_frame[workspace.name]["transform"]
        transform = copy.deepcopy(transform_frame)
        transform.header.frame_id = "base_link"
        transform.header.stamp = stamp
        tmp_buffer.set_transform(transform, "default_authority")

        # Correcting yaw to avoid out of reach targets
        t = tmp_buffer.lookup_transform("base_link", "object_base", rospy.Time(0))
        roll, pitch, yaw = transformations.euler_from_quaternion([t.transform.rotation.x, t.transform.rotation.y,
                                                                  t.transform.rotation.z, t.transform.rotation.w])
        if yaw_center is not None:
            if yaw < yaw_center - np.pi / 2:
                yaw += np.pi
            elif yaw > yaw_center + np.pi / 2:
                yaw -= np.pi
        t.header.stamp = rospy.Time.now()
        t.transform.rotation = Quaternion(*transformations.quaternion_from_euler(roll, pitch, yaw))
        self.__tf_buffer.set_transform(t, "default_authority")

    def set_grip(self, grip):
        """
        Updates the transform object_base -> tool_link_target in local tfBuffer
        :param grip:

        """
        if grip.transform.header.frame_id != "object_base":
            rospy.logerr("Poses Transform Handler - Grip transform need to have header frame 'object_base'")
            return False

        if grip.transform.child_frame_id != "tool_link_target":
            rospy.logerr("Poses Transform Handler - Grip transform need to have child frame 'tool_link_target'")
            return False

        grip.transform.header.stamp = rospy.Time.now()

        self.__tf_buffer.set_transform(grip.transform, "default_authority")
        return True

    def get_object_base_transform(self):
        """
        Reads the transform base_link -> object_base from local tfBuffer

        :returns: transform base_link -> object_base
        """
        return self.__tf_buffer.lookup_transform("base_link", "object_base", rospy.Time(0))

    def get_gripping_transform(self):
        """
        Reads the transform base_link -> tool_link_target from local tfBuffer

        :returns: transform base_link -> tool_link_target
        """

        tmp_buffer = tf2_ros.Buffer()
        stamp = rospy.Time.now()

        t = self.__tf_buffer.lookup_transform("base_link", "object_base", rospy.Time(0))
        t.header.stamp = stamp
        tmp_buffer.set_transform(t, "default_authority")

        t = self.__tf_buffer.lookup_transform("object_base", "tool_link_target", rospy.Time(0))
        t.header.stamp = stamp
        tmp_buffer.set_transform(t, "default_authority")

        return tmp_buffer.lookup_transform("base_link", "tool_link_target", rospy.Time(0))

    def get_object_transform(self, x_off=0.0, y_off=0.0, z_off=0.0, roll_off=0.0, pitch_off=np.pi, yaw_off=np.pi):
        """
        Reads the transform base_link -> object_base from local tfBuffer

        :returns: transform base_link -> object_base
        """
        t_base_to_object = self.__tf_buffer.lookup_transform("base_link", "object_base", rospy.Time(0))
        t_object_to_pick = self.transform_from_euler(x_off,
                                                     y_off,
                                                     z_off,
                                                     roll_off,
                                                     pitch_off,
                                                     yaw_off,
                                                     "object_base",
                                                     "pick_target",
                                                     t_base_to_object.header.stamp)
        self.__tf_buffer.set_transform(t_object_to_pick, "default_authority")

        return self.__tf_buffer.lookup_transform("base_link", "pick_target", rospy.Time(0))

    def get_calibration_tip_position(self, robot_pose):
        """
        Retrieves the position of the calibration tip from a given robot pose.

        :param robot_pose: pose of the robot's tool_link
        :returns: xyz position of calibration tip in robot coordinates
        """
        stamp = rospy.Time.now()
        # First apply transform for robot pose
        base_link_to_tool_link = self.transform_from_euler(robot_pose.position.x,
                                                           robot_pose.position.y,
                                                           robot_pose.position.z,
                                                           robot_pose.rpy.roll,
                                                           robot_pose.rpy.pitch,
                                                           robot_pose.rpy.yaw,
                                                           "base_link",
                                                           "tool_link")
        base_link_to_tool_link.header.stamp = stamp
        self.__tf_buffer.set_transform(base_link_to_tool_link, "default_authority")

        # Getting calibration tip
        calibration_tip_grip = self.__grip_manager.read("default_Calibration_Tip")
        tool_link_to_calib_tip = calibration_tip_grip.transform
        tool_link_to_calib_tip.header.frame_id = "tool_link"
        tool_link_to_calib_tip.header.stamp = stamp
        tool_link_to_calib_tip.child_frame_id = "calibration_tip"

        self.__tf_buffer.set_transform(tool_link_to_calib_tip, "default_authority")
        base_link_to_calib_tip = self.__tf_buffer.lookup_transform("base_link", "calibration_tip", rospy.Time(0))

        calib_tip_position = base_link_to_calib_tip.transform.translation
        # rospy.loginfo("Base Link -> Tool Link : {}".format(base_link_to_tool_link.transform.translation))
        # rospy.loginfo("Base Link -> Calib Tip : {}".format(base_link_to_calib_tip.transform.translation))
        return calib_tip_position

    @staticmethod
    def transform_from_euler(x, y, z, roll, pitch, yaw, header_frame_id, child_frame_id, stamp=None):
        """
        Creates a new stamped transform from translation and euler-orientation

        :param x: x translation
        :param y: y translation
        :param z: z translation
        :param roll: orientation roll
        :param pitch: orientation pitch
        :param yaw: orientation yaw
        :param header_frame_id: transform from this frame
        :param child_frame_id: transform to this frame
        :param stamp: the timestamp of the transform. Default = current time

        :returns: transform
        """
        if stamp is None:
            stamp = rospy.Time.now()
        t = TransformStamped()
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        q = transformations.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        t.header.frame_id = header_frame_id
        t.header.stamp = stamp
        t.child_frame_id = child_frame_id

        return t

    def enable_debug(self):
        """
        Start publishing debug information on /tf and /visualization_marker for
        debugging using rviz. This will happen in a separate thread.
        """
        self.__debug_thread = threading.Thread(target=self.__debug_loop, name="Poses Transform Handler Debug Thread")
        self.__debug_thread.start()

    def disable_debug(self):
        """
        Stop publishing debug inforation
        """
        self.__debug_stop_event.set()
        if self.__debug_thread is not None:
            self.__debug_thread.join()

    def __debug_loop(self):
        """
        Debug loop that will run in a separate thread.
        (tfBuffer should be threadsafe)
        """
        broadcaster = tf2_ros.TransformBroadcaster()
        rviz_marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1000)
        rate = rospy.Rate(5)
        while not self.__debug_stop_event.is_set() and not rospy.is_shutdown():
            if self.__debug_current_ws is None:
                rospy.logerr("Poses Transform Handler - Could not publish debug tf, no workspace set.")
                rate.sleep()
                continue

            try:
                broadcaster.sendTransform(
                    self.__tf_buffer.lookup_transform("base_link", self.__debug_current_ws.name, rospy.Time(0)))
                broadcaster.sendTransform(
                    self.__tf_buffer.lookup_transform(self.__debug_current_ws.name, "object_base", rospy.Time(0)))
                broadcaster.sendTransform(
                    self.__tf_buffer.lookup_transform("object_base", "tool_link_target", rospy.Time(0)))
            except tf2_ros.LookupException as e:
                rospy.logerr("Poses Transform Handler - Could not publish debug tf: {}".format(e))

            for i in range(4):  # Iterate over the 4 markers defining the workspace
                msg = Marker()
                msg.header.frame_id = "base_link"
                msg.id = i
                msg.type = 2  # It correspond to a sphere which will be drawn

                msg.pose.position.x = self.__debug_current_ws.points[i][0]
                msg.pose.position.y = self.__debug_current_ws.points[i][1]
                msg.pose.position.z = self.__debug_current_ws.points[i][2]

                msg.scale.x = 0.005
                msg.scale.y = 0.005
                msg.scale.z = 0.005

                msg.color.r = 1.0 if i == 0 or i == 3 else 0.0
                msg.color.g = 1.0 if i == 1 or i == 3 else 0.0
                msg.color.b = 1.0 if i == 2 or i == 3 else 0.0
                msg.color.a = 1.0

                rviz_marker_pub.publish(msg)

        rate.sleep()
