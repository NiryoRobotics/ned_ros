from abc import ABC

import numpy as np


class AbstractStream(ABC):
    """
    Abstract class for a stream.
    """

    def __init__(self):
        super().__init__()

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

    @property
    def image(self) -> np.ndarray:
        """
        Get the latest image from the stream.
        """
        raise NotImplementedError("Subclasses should implement this method.")

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
