from enum import Enum, unique
# import cv2


# Different object color that can be requested to the visions functions in Modbus
# don't use 0, this is used to leave registers empty when no solution was found
@unique
class ColorRequest(Enum):
    ANY = -1
    BLUE = 1
    RED = 2
    GREEN = 3

    NONE = 0



# Different object shapes that can be requested to the visions functions in Modbus
@unique
class ShapeRequest(Enum):
    ANY = -1
    CIRCLE = 1
    SQUARE = 2
    TRIANGLE = 3

    NONE = 0

