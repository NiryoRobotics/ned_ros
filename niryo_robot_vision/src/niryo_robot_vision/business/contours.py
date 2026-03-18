from typing import Optional, Tuple

import cv2
import numpy as np

from .enums import ObjectShape


def find_biggest_contours(img, nb_contours_max=10, min_area=400):
    """
    Function to find the biggest contour in an image

    :param img: the image to process
    :type img: numpy.array
    :param nb_contours_max: maximal number of contours which will be returned. Note that since the contours are sorted
    by area, the squares will always be the first ones. Thus, if the number of squares is greater than nb_contours_max,
    no circles will be returned.
    :type nb_contours_max: int
    :param min_area: minimal area of the contours to be considered
    :type min_area: int
    :return: the biggest contours found
    :rtype: list[OpenCV Contour]
    """
    contours, _hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filtered_contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    return sorted(filtered_contours, key=cv2.contourArea, reverse=True)[:nb_contours_max]


def find_shape(mask: np.ndarray,
               shape: ObjectShape,
               ratio_threshold=0.85) -> Optional[Tuple[float, float, float, ObjectShape]]:
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

        moment = cv2.moments(contour)
        if moment["m00"] > 0:
            x = int(moment["m10"] / moment["m00"])
            y = int(moment["m01"] / moment["m00"])

            _, (width, height), angle = cv2.minAreaRect(contour)
        else:
            (x, y), (width, height), angle = cv2.minAreaRect(contour)
        if width < height:
            width, height = height, width
            angle += 90

        if height > 0:
            ratio = height / width
            nb_squares = np.round(width / height)

            # The Shift logic targets the outermost square of the concatenated block.
            # Using 'nb_squares >= 2' ensures we only shift if it's not a single square.
            if ratio < ratio_threshold and nb_squares >= 2:
                shift_distance = width * (1 - 1 / nb_squares) / 2.0

                rad = np.radians(angle)
                # Calculate the X and Y components of the shift vector
                dx = shift_distance * np.cos(rad)
                dy = shift_distance * np.sin(rad)
                # Determine the coordinates of both extremities
                x1, y1 = x + dx, y + dy
                x2, y2 = x - dx, y - dy
                # Compute squared distance from the origin (0, 0) for both ends
                dist1_sq = x1**2 + y1**2
                dist2_sq = x2**2 + y2**2
                # Select the extremity that is closest to the origin
                if dist1_sq < dist2_sq:
                    x, y = x1, y1
                else:
                    x, y = x2, y2

                # Gripper is always perpendicular to the width (the long axis)
                gripper_angle = angle + 90

                # Normalisation [-90, 90]
                angle = (gripper_angle + 90) % 180 - 90

            elif found_shape == ObjectShape.CIRCLE:
                angle = 0
            elif found_shape == ObjectShape.SQUARE:
                # modulo 90 to get a value between 0 to 90,
                # then shift (by adding and subtracting 45) to get a value between -45 and 45.
                # This is done to have the minimal angle of rotation of the square(respectively of the gripper),
                # since a square rotated by 45 degrees is the same as a square not rotated at all.
                angle = (angle + 45) % 90 - 45

        angle = np.radians(angle)

        return x, y, angle, found_shape

    return None
