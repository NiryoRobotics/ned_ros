import cv2
import numpy as np

from .enums import ColorThreshold, ObjectColor


def hsv_threshold(img: np.ndarray, color_threshold: ColorThreshold) -> np.ndarray:
    """
    Take BGR image and return
    according to values on HSV (Hue, Saturation, Value)
    Pixel will worth 1 if a pixel has a value between min_v and max_v for all channels

    :param img: image BGR if rgb_space = False
    :param color_threshold: The color to use to filter the image
    :return: threshold image
    :rtype: numpy.array
    """
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.GaussianBlur(hsv_frame, (5, 5), 0, dst=hsv_frame)

    mask = np.zeros_like(hsv_frame, shape=(*hsv_frame.shape[0:2], 1), dtype=np.uint8)
    for low_thresh, high_thresh in color_threshold.value:
        cv2.bitwise_or(mask, cv2.inRange(hsv_frame, low_thresh, high_thresh), dst=mask)

    # "opening followed by closing". It allows to remove small noises and fill small holes
    kernel = np.ones((3, 3), np.uint8)
    cv2.erode(mask, kernel, dst=mask, iterations=2)
    cv2.dilate(mask, kernel, dst=mask, iterations=2)
    return mask


def most_present_color(img: np.ndarray, x: float, y: float, sample_size: int = 3) -> ObjectColor:
    """
    Get the most present color in a small area around the center of the object
    :param img: The image to process
    :param x: The x coordinate of the center of the object
    :param y: The y coordinate of the center of the object
    :param sample_size: The size of the area to sample
    :return: The most present color in the area
    """
    x_int = int(x)
    y_int = int(y)
    colors_representation = np.mean(img[y_int - sample_size:y_int + sample_size,
                                        x_int - sample_size:x_int + sample_size],
                                    axis=(0, 1))
    most_present_channel = np.argmax(colors_representation)
    return ObjectColor(most_present_channel)
