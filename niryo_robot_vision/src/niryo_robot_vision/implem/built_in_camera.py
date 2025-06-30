import time
from dataclasses import dataclass
from threading import Thread, Event
from typing import Callable

import cv2
import numpy as np

import rospy
from .abstract_stream import AbstractStream


@dataclass
class CameraConfig:
    acquisition_rate: int
    mtx: np.ndarray
    dist: np.ndarray
    width_img: float
    height_img: float
    flip: bool = False

    def to_ros_camera_info(self):
        """Converts the camera configuration to a ROS message."""
        from sensor_msgs.msg import CameraInfo
        msg = CameraInfo()
        msg.width = int(self.width_img)
        msg.height = int(self.height_img)
        msg.K = self.mtx.flatten().tolist()
        msg.D = self.dist.flatten().tolist()
        return msg


class Rate:

    def __init__(self, hz: int):
        self.__interval = 1.0 / hz
        self.__last_time = time.time()

    def sleep(self):
        now = time.time()
        elapsed = now - self.__last_time
        if elapsed < self.__interval:
            time.sleep(self.__interval - elapsed)
        self.__last_time = now


class Stream(AbstractStream):

    def __init__(self, camera_index: int, config: CameraConfig, publish_frame_cb: Callable[[np.ndarray], None]):
        """
        Initializes the camera stream.
        :param camera_index: Index of the camera to use (0 for the first camera, 1 for the second, etc.)
        :param config: Camera configuration
        :param publish_frame_cb: Callback function to publish the frame
        """
        super().__init__(publish_frame_cb)
        self.__camera_index = camera_index
        self.__config = config

        self._frame = np.zeros((int(config.height_img), int(config.width_img), 3), dtype=np.uint8)
        self.__acquisition_thread = None
        self.__stop_acquisition = Event()
        self.__rate = Rate(config.acquisition_rate)

    def __del__(self):
        try:
            self.stop()
        except RuntimeError:
            pass

    @property
    def is_available(self) -> bool:
        """Checks if the camera is available."""
        if self.is_active:
            return True

        tmp_video_stream = cv2.VideoCapture(self.__camera_index)
        is_opened = tmp_video_stream.isOpened()
        tmp_video_stream.release()
        return is_opened

    def start(self):
        """Starts the video stream."""

        if self.__acquisition_thread is not None:
            raise RuntimeError("Video stream is already started")

        self.__stop_acquisition.clear()
        self.__acquisition_thread = Thread(target=self.__loop, daemon=True)
        self.__acquisition_thread.start()

    def stop(self):
        if self.__acquisition_thread is None:
            raise RuntimeError("Video stream is already stopped")

        self.__stop_acquisition.set()
        self.__acquisition_thread.join()
        self.__acquisition_thread = None
        self.__stop_acquisition.clear()

    def __process_frame(self, frame: np.ndarray):
        """Processes the frame (e.g., flips it)."""
        if self.__config.flip:
            cv2.flip(frame, -1, dst=frame)

        cv2.undistort(src=frame, cameraMatrix=self.__config.mtx, distCoeffs=self.__config.dist, dst=self._frame)
        self._post_process()

    def __loop(self):
        video_stream = cv2.VideoCapture(self.__camera_index)
        if not video_stream.isOpened():
            raise RuntimeError("Can't connect to the camera")

        # This loop ensure the stream always restart after an error
        while not self.__stop_acquisition.is_set():

            # Main acquisition loop
            while video_stream.isOpened():
                if self.__stop_acquisition.is_set():
                    video_stream.release()
                    break

                ret, frame = video_stream.read()
                if not ret:
                    rospy.logwarn('Failed to read frame from camera')
                    break

                self.__process_frame(frame)
                try:
                    self._publish_frame(self._frame)
                except Exception as e:
                    rospy.logerr(f'Failed to publish frame: {e}')
                self.__rate.sleep()

            # If we end up here, it means either the camera is not available anymore or we failed to get a frame
            while not video_stream.isOpened() or video_stream.grab() is False:
                if self.__stop_acquisition.is_set():
                    video_stream.release()
                    break

                time.sleep(1)
                video_stream.open(self.__camera_index)

    @property
    def is_active(self) -> bool:
        return self.__acquisition_thread is not None and not self.__stop_acquisition.is_set()
