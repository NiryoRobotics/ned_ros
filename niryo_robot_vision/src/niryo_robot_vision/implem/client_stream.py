from typing import Callable

import cv2
import numpy as np

import rospy
from sensor_msgs.msg import CompressedImage
from .abstract_stream import AbstractStream


class Stream(AbstractStream):

    def __init__(self, topic: str, publish_frame_cb: Callable[[np.ndarray], None]):
        super().__init__(publish_frame_cb)
        self.__subscriber = None
        self.__topic = topic
        self._frame = np.zeros((480, 640, 3), dtype=np.uint8)  # Default size, will be updated in the callback

    def start(self):
        if self.__subscriber is not None:
            self.__subscriber.unregister()
        self.__subscriber = rospy.Subscriber(self.__topic, CompressedImage, self.__image_callback, queue_size=1)

    def stop(self):
        if self.__subscriber is None:
            return
        self.__subscriber.unregister()
        self.__subscriber = None

    def __image_callback(self, msg: CompressedImage):
        self._frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        self._post_process()
        self._publish_frame(self._frame)

    @property
    def is_active(self) -> bool:
        """
        We assume that if there is a publisher on the topic, then the stream is active.
        :return: True if the stream is active, False otherwise
        """
        return self.is_available

    @property
    def is_available(self) -> bool:
        return self.__subscriber is not None and self.__subscriber.get_num_connections() > 0
