# Libs
import rospy
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Messages
from geometry_msgs.msg import TransformStamped, Pose, Quaternion, Point


class ArmTCPTransformHandlerException(Exception):
    pass


class ArmTCPTransformHandler:
    """
    This class uses a TransformListener to handle transforms related to the TCP.
    """

    def __init__(self):
        self.__tf_buffer = Buffer()
        self.__tf_listener = TransformListener(self.__tf_buffer)
        self.__static_broadcaster = StaticTransformBroadcaster()

    def ee_link_to_tcp_pose_target(self, pose, ee_link):
        try:
            transform_tcp_to_ee_link = self.__tf_buffer.lookup_transform(ee_link, "TCP", rospy.Time(0))
            transform_tcp_to_ee_link.header.frame_id = "ee_link_target"
            transform_tcp_to_ee_link.child_frame_id = "tcp_target"

            stamp = transform_tcp_to_ee_link.header.stamp
            transform_world_to_tcp_target = self.transform_from_pose(pose, "base_link", "ee_link_target", stamp)

            self.__tf_buffer.set_transform(transform_world_to_tcp_target, "default_authority")
            self.__tf_buffer.set_transform(transform_tcp_to_ee_link, "default_authority")

            ee_link_target_transform = self.__tf_buffer.lookup_transform("base_link", "tcp_target", stamp)

            return self.pose_from_transform(ee_link_target_transform.transform)
        except ArmTCPTransformHandlerException:
            self.set_empty_tcp_to_ee_link_transform(ee_link)
            return pose

    def tcp_to_ee_link_pose_target(self, pose, ee_link):
        try:
            transform_tcp_to_ee_link = self.__tf_buffer.lookup_transform("TCP", ee_link, rospy.Time(0))
            transform_tcp_to_ee_link.header.frame_id = "tcp_target"
            transform_tcp_to_ee_link.child_frame_id = "ee_link_target"

            stamp = transform_tcp_to_ee_link.header.stamp
            transform_world_to_tcp_target = self.transform_from_pose(pose, "base_link", "tcp_target", stamp)

            self.__tf_buffer.set_transform(transform_world_to_tcp_target, "default_authority")
            self.__tf_buffer.set_transform(transform_tcp_to_ee_link, "default_authority")

            ee_link_target_transform = self.__tf_buffer.lookup_transform("base_link", "ee_link_target", stamp)

            return self.pose_from_transform(ee_link_target_transform.transform)
        except ArmTCPTransformHandlerException:
            self.set_empty_tcp_to_ee_link_transform(ee_link)
            return pose

    def tcp_to_ee_link_position_target(self, position, ee_link):
        pose = Pose(position, Quaternion(0, 0, 0, 1))
        return self.tcp_to_ee_link_pose_target(pose, ee_link).position

    def tcp_to_ee_link_quaternion_target(self, quaternion, ee_link):
        pose = Pose(Point(0, 0, 0), quaternion)
        return self.tcp_to_ee_link_pose_target(pose, ee_link).orientation

    def tcp_to_ee_link_rpy_target(self, roll, pitch, yaw, ee_link):
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose = Pose(Point(0, 0, 0), Quaternion(qx, qy, qz, qw))
        pose = self.tcp_to_ee_link_pose_target(pose, ee_link)
        new_roll, new_pitch, new_yaw = euler_from_quaternion(*pose.orientation)
        return new_roll, new_pitch, new_yaw

    def set_empty_tcp_to_ee_link_transform(self, ee_link):
        ee_link_to_tcp_transform = self.transform_from_pose(Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                                                            ee_link, "TCP")
        self.__static_broadcaster.sendTransform(ee_link_to_tcp_transform)
        return ee_link_to_tcp_transform

    def lookup_transform(self, target_frame, source_frame, stamp=None):
        if stamp:
            return self.__tf_buffer.lookup_transform(target_frame, source_frame, stamp)

        return self.__tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0))

    def display_target_pose(self, pose, name="target_pose"):
        t = self.transform_from_pose(pose, "base_link", name)
        self.__static_broadcaster.sendTransform(t)

    @staticmethod
    def transform_from_pose(pose, parent_frame, child_frame, stamp=None):
        t = TransformStamped()
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation = pose.orientation

        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        if stamp:
            t.header.stamp = stamp

        return t

    @staticmethod
    def pose_from_transform(transform):
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation = transform.rotation
        return pose
