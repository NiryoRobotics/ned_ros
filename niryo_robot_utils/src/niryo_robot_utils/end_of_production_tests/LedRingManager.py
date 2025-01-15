import random
from threading import Event

import rospy

from niryo_robot_python_ros_wrapper import NiryoRosWrapper

from end_effector_interface.msg import EEButtonStatus

LED_RING_COLORS = [
    [255, 0, 0],  # red
    [0, 255, 0],  # green
    [0, 0, 255],  # blue
    [255, 255, 0],  # yellow
    [0, 255, 255],  # cyan
    [255, 0, 255],  # magenta
    [255, 255, 255],  # white
    [255, 50, 0],  # orange
]


class LedRingManager:
    from end_effector_interface.msg import EEButtonStatus

    def __init__(self, robot: NiryoRosWrapper):
        self.__robot = robot
        self.__led_ring_color = None
        self.__led_ring_colors = LED_RING_COLORS.copy()
        self.__stop = Event()
        self.__freemotion_button_subscriber = rospy.Subscriber(
            '/niryo_robot_hardware_interface/end_effector_interface/free_drive_button_status',
            EEButtonStatus,
            self.__freemotion_button_callback)
        self.__save_button_subscriber = rospy.Subscriber(
            '/niryo_robot_hardware_interface/end_effector_interface/save_pos_button_status',
            EEButtonStatus,
            self.__save_button_callback)
        self.__update_led_ring()

    @property
    def led_ring_color(self):
        return self.__led_ring_color

    def run(self):
        while not rospy.is_shutdown() and not self.__stop.is_set():
            rospy.sleep(0.1)
        self.__freemotion_button_subscriber.unregister()
        self.__save_button_subscriber.unregister()

    def __freemotion_button_callback(self, msg: EEButtonStatus):
        if msg.action == EEButtonStatus.SINGLE_PUSH_ACTION:
            self.__update_led_ring()

    def __save_button_callback(self, msg: EEButtonStatus):
        if msg.action == EEButtonStatus.SINGLE_PUSH_ACTION:
            self.__stop.set()

    def __update_led_ring(self):
        if self.__led_ring_colors == []:
            self.__led_ring_colors = LED_RING_COLORS.copy()
            # prevent the same color from being chosen twice in a row
            self.__led_ring_colors.remove(self.__led_ring_color)

        self.__led_ring_color = self.__led_ring_colors.pop(random.randint(0, len(self.__led_ring_colors) - 1))

        self.__robot.led_ring.snake(self.__led_ring_color, wait=False)
