import math
import numpy as np

import cv2

from .enums import MorphoType, KernelType, ColorHSV, font_scale_normal, thickness_small, font, WHITE, ORANGE
from .markers_detection import extract_img_markers, draw_markers


# Image Processing
def improve_mask(mask):
    """
    Improve binary mask with morphological operation

    :param mask: input mask to improve
    :type mask: numpy.array
    :return mask: mask improved
    :rtype mask: numpy.array
    """

    kernel = np.ones((3, 3))
    mask = cv2.erode(mask, kernel)
    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)
    mask = cv2.dilate(mask, kernel)
    return mask


def threshold_red(img_hsv):
    """
    Threshold hsv image of red component
    :param img_hsv: hsv image to be threshold
    :type img_hsv: numpy.array
    :rtype: numpy.array
    """
    threshold_low = [0, 85, 120], [35, 255, 255]
    threshold_high = [150, 100, 80], [180, 255, 255]
    red_low_img = cv2.inRange(img_hsv, tuple(threshold_low[0]), tuple(threshold_low[1]))
    red_high_img = cv2.inRange(img_hsv, tuple(threshold_high[0]), tuple(threshold_high[1]))
    red_img = cv2.bitwise_or(red_low_img, red_high_img)
    red_img = improve_mask(red_img)
    return red_img


def threshold_green(img_hsv):
    """
    Threshold hsv image of green component
    :param img_hsv: hsv image to be threshold
    :type img_hsv: numpy.array
    :rtype: numpy.array
    """
    threshold = [60, 30, 60], [95, 255, 255]
    green_img = cv2.inRange(img_hsv, tuple(threshold[0]), tuple(threshold[1]))
    green_img = improve_mask(green_img)
    return green_img


def threshold_blue(img_hsv):
    """
    Threshold hsv image of blue component
    :param img_hsv: hsv image to be threshold
    :type img_hsv: numpy.array
    :rtype: numpy.array
    """
    threshold = [100, 50, 100], [125, 255, 255]
    blue_img = cv2.inRange(img_hsv, tuple(threshold[0]), tuple(threshold[1]))
    blue_img = improve_mask(blue_img)
    return blue_img


def threshold_hsv(img, color_hsv):
    """
    Take BGR image (OpenCV imread result) and return thresholded image
    according to values on HSV (Hue, Saturation, Value)
    Pixel will worth 1 if a pixel has a value between min_v and max_v for all channels

    :param img: image BGR if rgb_space = False
    :type img: numpy.array
    :param color_hsv: HSV color space
    :type color_hsv: ColorHSV
    :return: threshold image
    :rtype: numpy.array
    """
    frame = cv2.GaussianBlur(img, (5, 5), 0)
    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # red threshold
    if color_hsv.value == 1:
        thresh_img = threshold_red(frame_hsv)
    # green threshold
    elif color_hsv.value == 2:
        thresh_img = threshold_green(frame_hsv)
    # blue threshold
    elif color_hsv.value == 3:
        thresh_img = threshold_blue(frame_hsv)
    # any
    else:
        thresh_red = threshold_red(frame_hsv)
        thresh_green = threshold_green(frame_hsv)
        thresh_blue = threshold_blue(frame_hsv)

        thresh_img = cv2.bitwise_or(thresh_red, thresh_green)
        thresh_img = cv2.bitwise_or(thresh_img, thresh_blue)

    return thresh_img


def morphological_transformations(im_thresh,
                                  morpho_type=MorphoType.CLOSE,
                                  kernel_shape=(5, 5),
                                  kernel_type=KernelType.ELLIPSE):
    """
    Take black & white image and apply morphological transformation

    :param im_thresh: Black & White Image
    :type im_thresh: numpy.array
    :param morpho_type: CLOSE/OPEN/ERODE/DILATE => See on OpenCV/Google if you do not know these words
    :type morpho_type: MorphoType
    :param kernel_shape: tuple corresponding to the size of the kernel
    :type kernel_shape: tuple[int]
    :param kernel_type: RECT/ELLIPSE/CROSS => see on OpenCV
    :type kernel_type: KernelType
    :return: image after processing
    :rtype: numpy.array
    """
    if not isinstance(morpho_type, MorphoType) or not isinstance(kernel_type, KernelType):
        raise TypeError

    kernel = cv2.getStructuringElement(kernel_type.value, kernel_shape)

    return cv2.morphologyEx(im_thresh, morpho_type.value, kernel)


# Contours
def get_contour_barycenter(contour):
    """
    Return barycenter of an OpenCV Contour

    :param contour:
    :type contour: OpenCV Contour
    :return: Barycenter X, Barycenter Y
    :rtype: int, int
    """
    moments = cv2.moments(contour)
    if moments['m00'] == 0:
        rect = cv2.boundingRect(contour)
        cx, cy, _, _ = rect
    else:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

    return cx, cy


def get_contour_angle(contour):
    """
    Return orientation of a contour according to the smallest side
    in order to be well oriented for gripper

    :param contour: contour
    :type contour: OpenCV Contour
    :return: Angle in radians
    :rtype: float
    """
    rotrect = cv2.minAreaRect(contour)
    angle = rotrect[-1]
    size1, size2 = rotrect[1][0], rotrect[1][1]
    ratio_size = float(size1) / float(size2)
    if 1.25 > ratio_size > 0.75:
        if angle < -45:
            angle = 90 + angle
    else:
        if size1 < size2:
            angle = angle + 180
        else:
            angle = angle + 90

        if angle > 90:
            angle = angle - 180

    return math.radians(angle)


def biggest_contours_finder(img, nb_contours_max=3):
    """
    Function to find the biggest contour in an binary image

    :param img: Binary Image
    :type img: numpy.array
    :param nb_contours_max: maximal number of contours which will be returned
    :type nb_contours_max: int
    :return: biggest contours found
    :rtype: list[OpenCV Contour]
    """
    contours = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[-2]
    if not contours:
        return None
    contours_area = list()
    for cnt in contours:
        contours_area.append(cv2.contourArea(cnt))
    biggest_contours = []
    le = len(contours_area)
    if nb_contours_max > le:
        nb_contours = le
        id_contours_sorted_init = list(range(nb_contours))
    else:
        nb_contours = nb_contours_max
        id_contours_sorted_init = np.argpartition(contours_area, -nb_contours)[-nb_contours:]
    id_contours_sorted = [x for x in sorted(id_contours_sorted_init, key=lambda idi: -contours_area[idi])]

    for i in range(nb_contours):
        id_used = id_contours_sorted[i]

        if contours_area[id_used] < 400:
            break

        biggest_contours.append(contours[id_used])
    return biggest_contours


def extract_img_workspace(img, workspace_ratio=1.0):
    """
    Extract working area from an image thanks to 4 Niryo's markers

    :param img: OpenCV image which contain 4 Niryo's markers
    :type img: numpy.array
    :param workspace_ratio: Ratio between the width and the height of the area represented by the markers
    :type workspace_ratio: float
    :return: extracted and warped working area image
    :rtype: numpy.array
    """
    return extract_img_markers(img, workspace_ratio)


# -- IMAGE EDITING
def compress_image(img, quality=90):
    """
    Compress OpenCV image

    :param img: OpenCV Image
    :type img: numpy.array
    :param quality: integer between 1 - 100. The higher it is, the less information will be lost,
     but the heavier the compressed image will be
    :type quality: int
    :return: status & string representing compressed image
    :rtype: bool, str
    """
    result, encimg = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not result:
        return False, None

    return True, np.array(encimg).tobytes()


def add_annotation_to_image(img, text, write_on_top=True):
    """
    Add Annotation to an image

    :param img: Image
    :type img: numpy.array
    :param text: text string
    :type text: str
    :param write_on_top: if you write the text on top
    :type write_on_top: bool
    :return: img with text written on it
    :rtype: numpy.array
    """
    font_scale_used = font_scale_normal
    thickness_used = thickness_small
    h_im, w_im = img.shape[:2]
    (text_width, text_height), baseline = cv2.getTextSize(text, font, fontScale=font_scale_used,
                                                          thickness=thickness_used)
    text_true_height = text_height + baseline
    if write_on_top:
        cv2.rectangle(img, (0, 0), (int(text_width * 1.1), int(text_true_height * 1.35)), WHITE, cv2.FILLED)
        cv2.putText(img,
                    text, (int(text_width * 0.05), int(text_true_height * 1.2 - baseline)),
                    font,
                    font_scale_used,
                    ORANGE,
                    thickness_used)
    else:
        cv2.rectangle(img, (int(text_width * 1.1), h_im - int(text_true_height * 1.35)), (0, h_im), WHITE, cv2.FILLED)
        cv2.putText(img, text, (int(text_width * 0.05), h_im - baseline), font, font_scale_used, ORANGE, thickness_used)
    return img


# DEBUG


def debug_threshold_color(img, color_hsv):
    """
    Return masked image to see the effect of color threshold

    :param img: OpenCV image
    :type img: numpy.array
    :param color_hsv: Color used for debug
    :type color_hsv: ColorHSV
    :return: Masked image
    :rtype: numpy.array
    """
    thresh_img = threshold_hsv(img, color_hsv)
    im_morph = morphological_transformations(thresh_img, MorphoType.OPEN, kernel_shape=(7, 7))
    return cv2.bitwise_and(img, img, mask=im_morph)


def debug_markers(img, workspace_ratio=1.0):
    """
    Display detected markers on an image

    :param img: OpenCV image which contain Niryo's markers
    :type img: numpy.array
    :param workspace_ratio: Ratio between the width and the height of the area represented by the markers
    :type workspace_ratio: float
    :return: (status, annotated image)
    :rtype: numpy.array
    """
    return draw_markers(img, workspace_ratio)
