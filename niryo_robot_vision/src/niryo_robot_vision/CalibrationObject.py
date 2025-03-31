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
    def set_from_raw(cls, raw_mtx, raw_dist):
        mtx = np.reshape(raw_mtx, (3, 3))
        dist = np.expand_dims(raw_dist, axis=0)
        return cls(mtx, dist)

    @classmethod
    def set_from_dict(cls, d: dict):

        mtx = np.reshape(d["mtx"], (3, 3))
        dist = np.expand_dims(d["dist"], axis=0)
        return cls(mtx, dist)

    @classmethod
    def set_empty(cls):
        mtx = np.zeros((3, 3), dtype=float)
        dist = []
        return cls(mtx, dist)

    def __str__(self):
        big_string = "mtx\n" + str(self.__mtx) + "\n"
        big_string += "dist\n" + str(self.__dist) + "\n"
        return big_string

    def undistort_image(self, img):
        return cv2.undistort(src=img, cameraMatrix=self.__mtx, distCoeffs=self.__dist, newCameraMatrix=None)

    def get_camera_info(self):
        """
        Return value for ROS CameraInfo message
        """
        return self.__mtx, self.__dist

    def is_set(self):
        return self.__fx != 0.0 and self.__fy != 0.0 and self.__cx != 0.0 and self.__cy != 0.0
