from dataclasses import dataclass
from enum import Enum

import numpy as np


class ColorThreshold(Enum):
    """
    Class to represent the color thresholds used in the image processing. Colors are defined in HSV space.
    """
    RED = [((0, 85, 120), (35, 255, 255)), ((150, 100, 80), (180, 255, 255))]
    GREEN = [((60, 30, 60), (95, 255, 255))]
    BLUE = [((100, 65, 125), (125, 255, 255))]
    ANY = [*RED, *GREEN, *BLUE]


class ObjectColor(Enum):
    """
    Class to represent the color of the object. Their values are indexed according to the BGR channels.
    """
    ANY = -1
    BLUE = 0
    GREEN = 1
    RED = 2


Color2Threshold = {
    ObjectColor.RED: ColorThreshold.RED,
    ObjectColor.GREEN: ColorThreshold.GREEN,
    ObjectColor.BLUE: ColorThreshold.BLUE,
    ObjectColor.ANY: ColorThreshold.ANY
}


class ObjectShape(Enum):
    ANY = 0
    SQUARE = 4
    CIRCLE = np.inf

    @classmethod
    def from_nb_sides(cls, nb_sides: int):
        if nb_sides < cls.SQUARE.value:
            raise ValueError(f"Invalid number of sides: {nb_sides}")
        elif nb_sides == cls.SQUARE.value:
            return cls.SQUARE
        else:
            return cls.CIRCLE


@dataclass
class Object:
    """
    Class to represent an object detected in the scene.
    """

    shape: ObjectShape
    color: ObjectColor
    x: float
    y: float
    yaw: float
