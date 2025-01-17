from typing import Dict
import re
import rospy
from abc import ABC, abstractmethod

from niryo_robot_python_ros_wrapper import NiryoRosWrapper

from std_msgs.msg import Int32

WAIT_FOR_BUTTON_TIMEOUT = 600
use_sound = True

# Dictionary which replace some words by their phonetic equivalent to improve the TTS pronunciation
# The key is a word (can be a regex) to replace and the value is the replacement
_accents_dictionary = {r'free-? ?motion': 'freemoshion'}

_ned2_offsets = (
    (0.14, 0.14),
    (0.26, 0.07),
    (0.05, 0.07),
    (0.07, 0.07),
    (0.09, 0.09),
    (0.02, 0.02),
)


def say(robot: NiryoRosWrapper, text: str):
    for key, value in _accents_dictionary.items():
        text = re.sub(key, value, text)

    if not use_sound:
        rospy.loginfo(text)
        return
    robot.sound.say(text, 1)


def parse_duration(value: str) -> rospy.Duration:
    """
    Parse a string into a rospy.Duration object.
    Format: {number}{unit}, e.g., '10s', '5m', '2h', '1d'.
    Supported units:
      - s: seconds
      - m: minutes
      - h: hours
      - d: days
    """
    try:
        units = {'s': 1, 'm': 60, 'h': 3600, 'd': 86400}  # Seconds per unit
        return rospy.Duration(sum(units[unit] * int(num) for num, unit in re.findall(r'(\d+)([smhd])', value)))
    except (ValueError, IndexError):
        raise TypeError(
            f"Invalid duration format: '{value}'. Expected format: '{{number}}{{unit}}', e.g., '10s', '5m', '2h', '1d'."
        )


class BaseTest(ABC):

    def __init__(self, robot: NiryoRosWrapper):
        self._robot = robot

    @property
    def name(self) -> str:
        return ' '.join(re.findall(r'([A-Z]+[a-zéè]*|\d+)', self.__class__.__name__))

    def pre_test(self):
        """Optional method. Subclasses can override this if needed."""
        pass

    @abstractmethod
    def __call__(self, *args, **kwargs):
        raise NotImplementedError

    def post_test(self):
        """Optional method. Subclasses can override this if needed."""
        pass

    def _wait_for_save_button(self, msg=None):
        if msg is None:
            msg = 'Appuyez sur le bouton save pour continuer'

        self._say(msg)

        rospy.wait_for_message('/niryo_robot/blockly/save_current_point', Int32, timeout=WAIT_FOR_BUTTON_TIMEOUT)

    def _wait_for_custom_button(self, msg=None):
        if msg is None:
            msg = 'Appuyez sur le bouton custom pour continuer'

        self._say(msg)
        self._robot.custom_button.wait_for_any_action(timeout=WAIT_FOR_BUTTON_TIMEOUT)

    def _wait_for_freemotion_button(self, msg=None):
        if msg is None:
            msg = 'Appuyez sur le bouton freemotion pour continuer'

        self._say(msg)
        while not self._robot.get_learning_mode():
            rospy.sleep(0.1)

    def _get_joints_limits(self) -> Dict[int, Dict[str, float]]:
        """
        Get the joints limits of the robot. The limits are reduced to avoid hitting the mechanical limits
        :return: The limits with the following format: {joint_number: {'min': min_value, 'max': max_value}}
        """
        if self._robot.get_hardware_version() == 'ned2':
            offsets = _ned2_offsets
        else:
            offsets = ((0, 0), ) * len(self._robot.get_joints())

        return {
            int(joint_name[-1]): {
                'min': joints_limits['min'] + offset_min, 'max': joints_limits['max'] - offset_max
            }
            for (joint_name,
                 joints_limits), (offset_min,
                                  offset_max) in zip(self._robot.get_axis_limits()[1]['joint_limits'].items(), offsets)
        }

    def _say(self, txt: str):
        say(self._robot, txt)
