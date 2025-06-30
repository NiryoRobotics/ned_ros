from abc import ABC
from typing import Callable

import numpy as np

from ..business.image_processing import post_process


class AbstractStream(ABC):
    """
    Abstract class for a stream.
    """

    def __init__(self, publish_frame_cb: Callable[[np.ndarray], None]):
        super().__init__()
        self._frame = None  # Unstable, change during the processing pipeline.
        self._image = None  # Stable. Always contain a processed image.
        self.brightness = 1.0
        self.contrast = 1.0
        self.saturation = 1.0
        self._publish_frame = publish_frame_cb

    def start(self):
        """
        Start the stream.
        """
        raise NotImplementedError("Subclasses should implement this method.")

    def stop(self):
        """
        Stop the stream.
        """
        raise NotImplementedError("Subclasses should implement this method.")

    def get_image(self, copy=False):
        """
        Get the current image from the stream.

        :param copy: If True, return a copy of the image. If False, return the original image.
        :return: The current image.
        """
        if self.image is None:
            return None

        return self.image.copy() if copy else self.image

    @property
    def is_active(self) -> bool:
        """
        Check if the stream is active.
        """
        raise NotImplementedError("Subclasses should implement this method.")

    @property
    def is_available(self) -> bool:
        """
        Check if the stream is available.
        """
        raise NotImplementedError("Subclasses should implement this method.")

    def _post_process(self):
        """
        Post-process the image.
        """
        post_process(self._frame, self.brightness, self.contrast, self.saturation, dst=self._frame)
        self.image = self._frame.copy()
