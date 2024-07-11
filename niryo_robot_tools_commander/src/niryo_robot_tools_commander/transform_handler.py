#!/usr/bin/env python

# Libs
import rospy
import copy
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Messages
from niryo_robot_msgs.msg import CommandStatus
from geometry_msgs.msg import TransformStamped, Quaternion

from niryo_robot_tools_commander.msg import TCP

# Services
from niryo_robot_msgs.srv import SetBool, Trigger
from niryo_robot_tools_commander.srv import SetTCP


class ToolTransformHandler:
    """
    This class uses a tfBuffer to handle transforms related to the tools.
    """

    def __init__(self):
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer)
        self.__static_broadcaster = StaticTransformBroadcaster()

        self.__tool_transform = self.empty_transform()
        self.__tcp_transform = self.empty_transform()
        self.__enable_tcp = False

        # Publisher
        self.__tcp_publisher = rospy.Publisher('~tcp', TCP, queue_size=10, latch=True)
        self.__tcp_v2_publisher = rospy.Publisher('~tcp_v2', TCP, queue_size=10, latch=True)
        rospy.Timer(rospy.Duration.from_sec(0.5), self.__send_tcp_transform)

        # Services
        rospy.Service('~set_tcp', SetTCP, self.__callback_set_tcp)
        rospy.Service('~reset_tcp', Trigger, self.__callback_reset_tcp)
        rospy.Service('~enable_tcp', SetBool, self.__callback_enable_tcp)

    def __callback_set_tcp(self, req):
        self.__enable_tcp = True

        t = self.empty_transform()

        if req.orientation == Quaternion():
            t.transform.rotation = Quaternion(*quaternion_from_euler(req.rpy.roll, req.rpy.pitch, req.rpy.yaw))
        else:
            t.transform.rotation = req.orientation

        t.transform.translation.x = req.position.x
        t.transform.translation.y = req.position.y
        t.transform.translation.z = req.position.z

        if req.tcp_version != req.DH_CONVENTION:
            t.transform.translation.x = req.position.z
            t.transform.translation.y = -req.position.y
            t.transform.translation.z = req.position.x

        self.set_tcp(t)
        return CommandStatus.SUCCESS, "Success"

    def __callback_reset_tcp(self, _):
        self.set_tcp(copy.deepcopy(self.__tool_transform))
        return CommandStatus.SUCCESS, "Success"

    def __callback_enable_tcp(self, req):
        self.__enable_tcp = req.value
        self.__send_tcp_transform(None)
        self.__publish_tcp_transform()
        return CommandStatus.SUCCESS, "Success"

    def set_tool(self, tool_transform):
        """
        Updates the transform object_base -> tool_link_target in local tfBuffer
        :param grip:

        """
        self.__tool_transform = tool_transform
        return self.set_tcp(copy.deepcopy(self.__tool_transform))

    def set_tcp(self, tcp_transform):
        """
        Updates the transform object_base -> tool_link_target in local tfBuffer
        :param grip:

        """
        self.__tcp_transform = tcp_transform
        self.__tcp_transform.header.stamp = rospy.Time.now()
        self.__tf_buffer.set_transform(self.__tcp_transform, "default_authority")
        self.__send_tcp_transform(None)
        self.__publish_tcp_transform()
        return True

    def __publish_tcp_transform(self):
        msg = TCP()
        msg.enabled = self.__enable_tcp

        msg.position.x = self.__tcp_transform.transform.translation.x
        msg.position.y = self.__tcp_transform.transform.translation.y
        msg.position.z = self.__tcp_transform.transform.translation.z
        msg.orientation = self.__tcp_transform.transform.rotation
        msg.rpy.roll, msg.rpy.pitch, msg.rpy.yaw = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])

        self.__tcp_v2_publisher.publish(msg)

        msg_v1 = TCP()
        msg_v1.enabled = self.__enable_tcp
        msg_v1.position.x = self.__tcp_transform.transform.translation.z
        msg_v1.position.y = -self.__tcp_transform.transform.translation.y
        msg_v1.position.z = self.__tcp_transform.transform.translation.x
        msg_v1.rpy = msg.rpy
        msg_v1.orientation = msg.orientation

        self.__tcp_publisher.publish(msg_v1)

    def __send_tcp_transform(self, _):
        t = self.__tcp_transform if self.__enable_tcp else self.empty_transform()
        t.header.stamp = rospy.Time.now()

        self.__static_broadcaster.sendTransform(t)

    @staticmethod
    def transform_from_dict(dict_):
        t = TransformStamped()
        t.transform.translation.x = dict_["translation"][0]
        t.transform.translation.y = dict_["translation"][1]
        t.transform.translation.z = dict_["translation"][2]

        t.transform.rotation.x = dict_["quaternion"][0]
        t.transform.rotation.y = dict_["quaternion"][1]
        t.transform.rotation.z = dict_["quaternion"][2]
        t.transform.rotation.w = dict_["quaternion"][3]

        t.header.frame_id = "tool_link"
        t.child_frame_id = "TCP"

        return t

    @staticmethod
    def empty_transform():
        t = TransformStamped()
        t.transform.rotation = Quaternion(0, 0, 0, 1)
        t.header.frame_id = "tool_link"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "TCP"
        return t
