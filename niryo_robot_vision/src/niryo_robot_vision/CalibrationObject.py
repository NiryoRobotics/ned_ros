import cv2
import numpy as np


class CalibrationObject:
    """
    Class containing intrinsic parameters of the camera used
    """

    def __init__(self, mtx, dist):
        self.__mtx = mtx
        self.__dist = dist

        self.__fx = self.__mtx[0][0]
        self.__fy = self.__mtx[1][1]
        self.__cx = self.__mtx[0][2]
        self.__cy = self.__mtx[1][2]

    @classmethod
    def set_from_values(cls, mtx, dist):
        return cls(mtx, dist)

    @classmethod
    def set_from_yaml(cls, yaml_file):
        mtx = np.reshape(yaml_file["mtx"], (3, 3))
        dist = np.expand_dims(yaml_file["dist"], axis=0)
        return cls(mtx, dist)

    @classmethod
    def set_empty(cls):
        mtx = np.zeros((3, 3), dtype=np.float)
        dist = []
        return cls(mtx, dist)

    def __str__(self):
        big_string = "mtx\n" + str(self.__mtx) + "\n"
        big_string += "dist\n" + str(self.__dist) + "\n"
        return big_string

    def get_center_position(self):
        return tuple([self.__cx, self.__cy])

    def get_center_position_int(self):
        return tuple([int(round(self.__cx)), int(round(self.__cy))])

    def get_intrinsic_parameters(self):
        return {
            "fx": self.__fx,
            "fy": self.__fy,
            "cx": self.__cx,
            "cy": self.__cy,
        }

    def get_cam_mtx_and_dist_coefs(self):
        return self.__mtx, self.__dist

    def undistort_image(self, img):
        return cv2.undistort(src=img, cameraMatrix=self.__mtx,
                             distCoeffs=self.__dist, newCameraMatrix=None)

    def meters_to_pixels(self, length, width, z_offset):
        u = (length / z_offset) * self.__fx
        v = (width / z_offset) * self.__fy

        return tuple([u, v])

    def get_camera_info(self):
        """
        Return value for ROS CameraInfo message
        """
        return self.__mtx, self.__dist

    def is_set(self):
        return self.__fx != 0.0 and self.__fy != 0.0 and self.__cx != 0.0 and self.__cy != 0.0
