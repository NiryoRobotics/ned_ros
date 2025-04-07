from typing import Optional

import cv2
import numpy as np

from .enums import Object, ObjectColor, ObjectShape, Color2Threshold
from .markers import extract_img_markers
from .colors import hsv_threshold, most_present_color
from .contours import find_shape


class MarkersNotFoundError(Exception):
    pass


class NoShapeFoundError(Exception):
    pass


def detect_object(image: np.ndarray, color: ObjectColor, shape: ObjectShape,
                  workspace_ratio: float) -> Optional[Object]:
    """
    Detect an object in the image.
    :param image: The image to process.
    :param color: The color of the object to detect. If ObjectColor.ANY, the color will be determined after detection.
    :param shape: The shape of the object to detect. If ObjectShape.ANY, the shape will be determined after detection.
    :param workspace_ratio: The ratio of the workspace to use for detection. It should be between 0 and 1.
    :return:
    """
    # Extract working area image from markers
    im_work = extract_img_markers(image, workspace_ratio=workspace_ratio)
    if im_work is None:
        raise MarkersNotFoundError()

    # Get a black and white image of the image. white = wanted color, black = other colors
    mask = hsv_threshold(im_work, Color2Threshold[color])

    # Find the biggest shape in the image
    found_shape = find_shape(mask, shape)
    if found_shape is None:
        raise NoShapeFoundError()

    x, y, yaw, shape = found_shape

    # If we asked for any color, determine the color of the found object
    if color == ObjectColor.ANY:
        color = most_present_color(image, x, y)

    found_object = Object(shape=shape, color=color, x=x, y=y, yaw=yaw)
    found_object.x = found_object.x / im_work.shape[1]
    found_object.y = found_object.y / im_work.shape[0]

    return found_object


def __is_factor_significant(factor: float) -> bool:
    """
    Check if the factor is significant. A factor is considered significant if it has at least 0.1 difference from 1.0.
    :param factor:
    :return: True if the factor is significant, False otherwise.
    """
    return not np.isclose(factor, 1.0, rtol=0.01)


def post_process(image: np.ndarray,
                 brightness_factor: float,
                 contrast_factor: float,
                 saturation_factor: float,
                 dst: np.ndarray = None) -> np.ndarray:
    """
    Post-process the image to adjust brightness, contrast, and saturation.
    :param image: The image to process
    :param brightness_factor: The brightness factor to apply
    :param contrast_factor: The contrast factor to apply
    :param saturation_factor: The saturation factor to apply
    :param dst: The destination image (optional)
    :return: The processed image
    """
    if dst is None:
        dst = image.copy()

    if __is_factor_significant(brightness_factor) or __is_factor_significant(contrast_factor):
        # Adjust contrast and brightness
        alpha = contrast_factor
        beta = (brightness_factor - 1) * 255
        cv2.addWeighted(dst, alpha, dst, 0, beta, dst)

    if __is_factor_significant(saturation_factor):
        # Adjust saturation
        cv2.cvtColor(dst, cv2.COLOR_BGR2HSV, dst=dst)
        dst[..., 1] = cv2.multiply(dst[..., 1], saturation_factor)
        return cv2.cvtColor(dst, cv2.COLOR_HSV2BGR, dst=dst)

    return dst
