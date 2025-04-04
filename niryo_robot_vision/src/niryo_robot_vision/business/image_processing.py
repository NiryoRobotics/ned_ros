from typing import Optional

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
