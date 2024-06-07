from typing import Union

from .Pose import Pose
from .JointsPosition import JointsPosition

RobotPosition = Union[JointsPosition, Pose]
