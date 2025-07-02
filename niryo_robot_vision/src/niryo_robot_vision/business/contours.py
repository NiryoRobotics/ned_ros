from typing import Optional, Tuple

import cv2
import numpy as np

from .enums import ObjectShape


def find_biggest_contours(img, nb_contours_max=3, min_area=400):
    """
    Function to find the biggest contour in an image

    :param img: the image to process
    :type img: numpy.array
    :param nb_contours_max: maximal number of contours which will be returned
    :type nb_contours_max: int
    :param min_area: minimal area of the contours to be considered
    :type min_area: int
    :return: the biggest contours found
    :rtype: list[OpenCV Contour]
    """
    contours, _hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    return sorted(filtered_contours, key=cv2.contourArea, reverse=True)[:nb_contours_max]


def find_shape(mask: np.ndarray, shape: ObjectShape) -> Optional[Tuple[float, float, float, ObjectShape]]:
    """
    Function to extract the biggest shape from a threshed image.
    :param mask: The image to process
    :param shape: The shape to extract
    :return: The object corresponding to the biggest shape found
    """

    contours = find_biggest_contours(mask)
    if contours is None:
        return None

    for contour in contours:
        peri = cv2.arcLength(contour, True)
        approx_n_side = len(cv2.approxPolyDP(contour, 0.035 * peri, True))

        try:
            found_shape = ObjectShape.from_nb_sides(approx_n_side)
        except ValueError:
            return None

        if found_shape != shape and shape != ObjectShape.ANY:
            continue

        (x, y), _size, angle = cv2.minAreaRect(contour)

        if found_shape == ObjectShape.CIRCLE:
            angle = 0

        angle = np.radians(angle)

        return x, y, angle, found_shape

    return None
