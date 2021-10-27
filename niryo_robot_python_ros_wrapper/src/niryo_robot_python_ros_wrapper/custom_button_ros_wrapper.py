from threading import Lock, Event
import rospy

from ros_wrapper_enums import ButtonAction

from end_effector_interface.msg import EEButtonStatus

from niryo_robot_msgs.msg import CommandStatus


class CustomButtonRosWrapperException(Exception):
    pass


def check_ned2_version(func):
    """
    Decorator that check the robot version
    """

    def wrap(*args, **kwargs):
        robot_instance = args[0]
        if robot_instance.hardware_version != 'ned2':
            raise CustomButtonRosWrapperException(
                "Error Code : {}\nMessage : Wrong robot hardware version, feature only available on Ned2".format(
                    CommandStatus.BAD_HARDWARE_VERSION))

        return func(*args, **kwargs)

    return wrap


class CustomButtonRosWrapper:
    def __init__(self, hardware_version='ned2'):
        self.__hardware_version = hardware_version

        self.__action_lock = Lock()
        self.__action_events = {
            ButtonAction.HANDLE_HELD_ACTION: Event(),
            ButtonAction.LONG_PUSH_ACTION: Event(),
            ButtonAction.SINGLE_PUSH_ACTION: Event(),
            ButtonAction.DOUBLE_PUSH_ACTION: Event(),
            ButtonAction.NO_ACTION: Event(),
        }

        self.__custom_button_state = EEButtonStatus.NO_ACTION

        self.__custom_button_topic = rospy.Subscriber(
            '/niryo_robot_hardware_interface/end_effector_interface/custom_button_status',
            EEButtonStatus, self.__callback_custom_pos_button_status)

    @property
    def hardware_version(self):
        return self.__hardware_version

    @check_ned2_version
    @property
    def state(self):
        return self.__custom_button_state

    @check_ned2_version
    def is_pressed(self):
        return self.__custom_button_state != EEButtonStatus.NO_ACTION

    @check_ned2_version
    def wait_for_action(self, action, timeout=0):
        return self.__wait(action, timeout)

    @check_ned2_version
    def wait_for_any_action(self, timeout=0):
        return self.__wait_any(timeout)

    @check_ned2_version
    def get_and_wait_press_duration(self, timeout=0):
        return self.__get_press_time(timeout)

    def __clear(self):
        with self.__action_lock:
            for a_event in self.__action_events.values():
                a_event.clear()

    def __set(self, action):
        self.__custom_button_state = action
        if action in self.__action_events:
            with self.__action_lock:
                self.__action_events[action].set()

    def __wait(self, action, timeout=0):
        if action not in self.__action_events:
            return False

        self.__action_events[action].clear()

        start_time = rospy.Time.now()
        while not self.__action_events[action].is_set() and not rospy.is_shutdown():
            self.__action_events[action].wait(0.1)

            if self.__action_events[action].is_set():
                return True
            elif 0 < timeout < (rospy.Time.now() - start_time).to_sec():
                return False

        if self.__action_events[action].is_set():
            return True

        return False

    def __wait_any(self, timeout=0):
        start_time = rospy.Time.now()
        self.__clear()
        if not self.__wait(ButtonAction.HANDLE_HELD_ACTION, timeout=timeout):
            return ButtonAction.NO_ACTION

        while not rospy.is_shutdown():
            if 0 < timeout < (rospy.Time.now() - start_time).to_sec():
                break

            for action_name in self.__action_events:
                if action_name not in [ButtonAction.NO_ACTION, ButtonAction.HANDLE_HELD_ACTION] \
                        and self.__action_events[action_name].is_set():
                    return action_name

            rospy.sleep(0.1)

        return ButtonAction.HANDLE_HELD_ACTION

    def __get_press_time(self, timeout=0):
        if not self.__wait(ButtonAction.HANDLE_HELD_ACTION, timeout=timeout):
            return 0

        pressed_time = rospy.Time.now()

        if not self.__wait(ButtonAction.NO_ACTION):
            return 0

        return (rospy.Time.now() - pressed_time).to_sec()

    def __callback_custom_pos_button_status(self, custom_button_status):
        self.__set(custom_button_status.action)
