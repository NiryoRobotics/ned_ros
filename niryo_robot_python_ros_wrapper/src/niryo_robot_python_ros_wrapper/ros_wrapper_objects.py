from threading import Lock, Event
import rospy

from ros_wrapper_enums import ButtonAction

from end_effector_interface.msg import EEButtonStatus


class CustomButtonObject:
    def __init__(self):
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

    def __callback_custom_pos_button_status(self, custom_button_status):
        self.set(custom_button_status.action)

    @property
    def state(self):
        return self.__custom_button_state

    @property
    def is_pressed(self):
        return self.__custom_button_state != EEButtonStatus.NO_ACTION

    def clear(self):
        with self.__action_lock:
            for a_event in self.__action_events.values():
                a_event.clear()

    def set(self, action):
        self.__custom_button_state = action
        if action in self.__action_events:
            with self.__action_lock:
                self.__action_events[action].set()

    def wait(self, action, timeout=0):
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

    def wait_any(self, timeout=0):
        start_time = rospy.Time.now()
        self.clear()
        if not self.wait(ButtonAction.HANDLE_HELD_ACTION, timeout=timeout):
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

    def get_press_time(self, timeout=0):
        if not self.wait(ButtonAction.HANDLE_HELD_ACTION, timeout=timeout):
            return 0

        pressed_time = rospy.Time.now()

        if not self.wait(ButtonAction.NO_ACTION):
            return 0

        return (rospy.Time.now() - pressed_time).to_sec()
