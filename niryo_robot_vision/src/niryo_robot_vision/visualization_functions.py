import numpy as np

import niryo_robot_poses_handlers.transform_functions as transformations


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
    convention_rotation = np.array([[0, -1, 0, 0], [-1, 0, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    object_rotation = transformations.concatenate_matrices(workspace["matrix_rotation"],
                                                           camera_rotation,
                                                           convention_rotation)
    roll, pitch, yaw = transformations.euler_from_matrix(object_rotation)

    # Correcting yaw to avoid out of reach targets
    if yaw_center is not None:
        if yaw < yaw_center - np.pi / 2:
            yaw += np.pi
        elif yaw > yaw_center + np.pi / 2:
            yaw -= np.pi

    q = transformations.quaternion_from_euler(roll, pitch, yaw)

    return position, q
