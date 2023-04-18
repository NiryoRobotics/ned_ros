from enum import Enum, auto


class CollisionPolicy(Enum):
    HARD = auto()  # set the flag and raise an exception
    SOFT = auto()  # set the flag
