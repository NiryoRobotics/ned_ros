from enum import Enum, unique
import cv2

# -- Text Display
RED = (0, 0, 255)
PURPLE = (255, 0, 255)
ORANGE = (0, 127, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

font = cv2.FONT_HERSHEY_SIMPLEX
font_scale_big = 2.0
font_scale_normal = 0.7

thickness_big = 3
thickness_small = 2


# Values for HSV threshold
@unique
class ColorHSV(Enum):
    """
    COLOR TYPE
    """
    ANY = 0
    RED = 1
    GREEN = 2
    BLUE = 3


# Different object types handle by the API. The value corresponds to the object's number of sides
@unique
class ObjectType(Enum):
    SQUARE = 4
    CIRCLE = -1
    ANY = 0


@unique
class MorphoType(Enum):
    ERODE = cv2.MORPH_ERODE
    DILATE = cv2.MORPH_DILATE
    OPEN = cv2.MORPH_OPEN
    CLOSE = cv2.MORPH_CLOSE


@unique
class KernelType(Enum):
    RECT = cv2.MORPH_RECT
    ELLIPSE = cv2.MORPH_ELLIPSE
    CROSS = cv2.MORPH_CROSS
