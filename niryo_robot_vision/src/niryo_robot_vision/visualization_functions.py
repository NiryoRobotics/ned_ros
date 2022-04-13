import numpy as np

import niryo_robot_poses_handlers.transform_functions as transformations

from .enums import *

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA


def create_clear_all_marker(time):
    """
    Create marker for clearing all

    :param time: time ros
    """
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.id = 0
    marker.ns = "niryo_robot_vision"
    marker.action = marker.DELETEALL

    return marker


def create_clear_marker(id_marker, time):
    """
    Create marker for clearing

    :param id_marker: id of the marker
    :param time: time ros
    """
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.id = id_marker
    marker.ns = "niryo_robot_vision"
    marker.action = marker.DELETE

    return marker


def create_workspace(id_workspace, time, position, orientation, x_size, y_size, z_size):
    """
    Create workspace plane

    :param id_workspace: id of the marker
    :param time: time ros
    :param position: position of the marker
    :param orientation: orientation of the marker
    :param x_size: size in x of the object
    :param y_size: size in y of the object
    :param z_size: size in z of the object
    """
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.id = id_workspace
    marker.ns = "niryo_robot_vision"

    marker.action = marker.ADD
    marker.pose.position = Point(*position)
    marker.pose.position.x -= z_size * 2
    marker.pose.orientation = Quaternion(*orientation)
    marker.type = marker.CUBE
    marker.scale = Vector3(x_size, y_size, z_size)
    # marker.color = ColorRGBA(0.99, 0.99, 0.99, 1.0)
    marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

    return marker


def create_text_marker(id_text, text, time, position, orientation, size):
    """
    Create text marker

    :param id_text: id of the marker
    :param text: text in the marker
    :param time: time ros
    :param position: position of the marker
    :param orientation: orientation of the marker
    :param size: size of the text
    """
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.id = id_text
    marker.ns = "niryo_robot_vision"

    marker.action = marker.ADD
    marker.pose.position = Point(*position)
    marker.pose.position.z += 0.01
    marker.pose.orientation = Quaternion(*orientation)
    marker.type = marker.TEXT_VIEW_FACING
    marker.scale = Vector3(size, size, size)
    marker.color = ColorRGBA(0.0, 0.725, 0.910, 1.0)
    marker.text = text

    return marker


def create_marker_rviz(position, orientation, id_marker, time):
    """
    Create marker for the workspace

    :param position: position of the marker
    :param orientation: orientation of the marker
    :param id_marker: id of the marker
    :param time: time ros
    """
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    marker.id = id_marker
    marker.ns = "niryo_robot_vision"
    marker.type = marker.MESH_RESOURCE
    if id_marker == 1:
        marker.mesh_resource = "package://niryo_robot_vision/meshes/marker_1.dae"
    else:
        marker.mesh_resource = "package://niryo_robot_vision/meshes/marker_2.dae"
    marker.mesh_use_embedded_materials = True
    marker.action = marker.ADD
    marker.pose.position = Point(*position)
    marker.pose.position.z += 0.0005
    marker.pose.orientation = Quaternion(*orientation)
    marker.scale = Vector3(1, 1, 1)
    marker.color = ColorRGBA(1, 1, 1, 1)

    return marker


def publish_object_rviz(position, orientation, size, type_marker, color, id_marker, publisher, time, duration):
    """
    Create and publish object and marker object

    :param position: position of the marker
    :param orientation: orientation of the marker
    :param size: size of the object
    :param type_marker: type of the object(cylinder or square)
    :param color: color of the object
    :param id_marker: id of the marker
    :param publisher: ros publisher
    :param time: time ros
    :param duration: time to live
    """
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = time
    # marker.lifetime = duration
    marker.id = id_marker
    marker.ns = "niryo_robot_vision"

    marker.action = marker.ADD
    marker.pose.position = Point(*position)
    marker.pose.position.z += 0.005
    marker.pose.orientation = Quaternion(*orientation)
    if type_marker == ObjectType.SQUARE:
        marker.type = marker.CUBE
        marker.scale = Vector3(size[0], size[1], 0.01)
    else:
        marker.type = marker.CYLINDER
        marker.scale = Vector3(size[0], size[0], 0.01)

    marker.color.a = 1
    if color == "RED":
        marker.color.r = 1.0
    elif color == "BLUE":
        marker.color.b = 1.0
    elif color == "GREEN":
        marker.color.g = 1.0

    publisher.publish([marker])


def get_pose(workspace, x_rel, y_rel, yaw_rel, yaw_center=None):
    """
    Transform the pose in the workspace to the world pose

    :param workspace: dict of the workspace which contains name, matrix, ratio
    :param x_rel: object base x position relative to workspace
    :param y_rel: object base y position relative to workspace
    :param yaw_rel: object base rotation on z relative to workspace
    :param yaw_center: Avoid over rotation
    """

    position = np.dot(workspace["matrix_position"], np.array([x_rel, y_rel, 1]))
    camera_rotation = transformations.euler_matrix(0, 0, yaw_rel)
    # Here we correct the object orientation to be similar to base_link if
    # the object in on the ground. Not neccessarily needed to be honest...
    convention_rotation = np.array([[0, -1, 0, 0],
                                    [-1, 0, 0, 0],
                                    [0, 0, -1, 0],
                                    [0, 0, 0, 1]])

    object_rotation = transformations.concatenate_matrices(
        workspace["matrix_rotation"], camera_rotation, convention_rotation)
    roll, pitch, yaw = transformations.euler_from_matrix(object_rotation)

    # Correcting yaw to avoid out of reach targets
    if yaw_center is not None:
        if yaw < yaw_center - np.pi / 2:
            yaw += np.pi
        elif yaw > yaw_center + np.pi / 2:
            yaw -= np.pi

    q = transformations.quaternion_from_euler(roll, pitch, yaw)

    return position, q
