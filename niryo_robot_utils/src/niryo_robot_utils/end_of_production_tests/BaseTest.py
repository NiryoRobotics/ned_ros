import re
from abc import ABC, abstractmethod
import rospy

from niryo_robot_python_ros_wrapper import NiryoRosWrapper

from end_effector_interface.msg import EEButtonStatus

from .utils import say


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

    def __wait_for_end_effector_button(self, topic_name, button_name, text=None):
        button_pressed = False

        def callback(msg):
            nonlocal button_pressed
            button_pressed = msg.action != EEButtonStatus.NO_ACTION

        if text is None:
            text = f'Appuyez sur le bouton {button_name} pour continuer'
        else:
            text += f' puis appuyez sur le bouton {button_name} pour continuer'

        self._say(text)
        subscriber = rospy.Subscriber(f'/niryo_robot_hardware_interface/end_effector_interface/{topic_name}',
                                      EEButtonStatus,
                                      callback)
        while not rospy.is_shutdown() and not button_pressed:
            rospy.sleep(0.1)
        subscriber.unregister()

    def _wait_for_save_button(self, text=None):
        self.__wait_for_end_effector_button('save_pos_button_status', 'save', text)

    def _wait_for_custom_button(self, text=None):
        self.__wait_for_end_effector_button('custom_button_status', 'custom', text)

    def _wait_for_freemotion_button(self, text=None):
        self.__wait_for_end_effector_button('free_drive_button_status', 'freemotion', text)

    def _say(self, txt: str):
        say(self._robot, txt)
