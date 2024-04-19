# led_manager.py
# Copyright (C) 2017 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
from threading import Thread, Lock

# Command Status
from std_msgs.msg import Int8
from niryo_robot_msgs.msg import CommandStatus, HardwareStatus
from niryo_robot_msgs.srv import SetInt
from niryo_robot_rpi.srv import LedBlinker

from niryo_robot_system_api_client import system_api_client

from ..common.rpi_ros_utils import LedState
from .rpi_io_objects import Led

LED_OFF = 0
LED_BLUE = 1
LED_GREEN = 2
LED_BLUE_GREEN = 3
LED_RED = 4
LED_PURPLE = 5
LED_RED_GREEN = 6
LED_WHITE = 7


class LEDManager(object):

    def __init__(self, lock=None):
        self._red_led = None
        self._green_led = None
        self._blue_led = None

        rospy.sleep(0.1)
        self.state = LedState.OK
        self._led_state_topic = rospy.Publisher('/niryo_robot/rpi/led_state', Int8, latch=True, queue_size=10)

        self._blinker_thread = None
        self._activate_blinker = None
        self._blinker_frequency = None
        self._blinker_color = None
        self._blinker_stop_time = None

        self._set_led_state_server = rospy.Service('/niryo_robot/rpi/set_led_state',
                                                   SetInt,
                                                   self._callback_set_led_state)

        self._set_led_custom_blinker_server = rospy.Service('/niryo_robot_rpi/set_led_custom_blinker',
                                                            LedBlinker,
                                                            self._callback_set_led_custom_blinker)

        self.hardware_status_subscriber = rospy.Subscriber('/niryo_robot_hardware_interface/hardware_status',
                                                           HardwareStatus,
                                                           self._callback_hardware_status)

        self.wifi_status_timer = rospy.Timer(rospy.Duration(1), self.__callback_wifi_state)

        self.set_led_from_state(dxl_leds=True)

        if lock is None:
            lock = Lock()

        self._red_led = Led(lock, rospy.get_param("~led/r_pin"), "Led Red")
        self._green_led = Led(lock, rospy.get_param("~led/g_pin"), "Led Green")
        self._blue_led = Led(lock, rospy.get_param("~led/b_pin"), "Led Blue")

        self.set_led_from_state(dxl_leds=True)
        rospy.loginfo('LED manager has been started.')

    @staticmethod
    def set_dxl_leds(color):
        leds = 0
        if color == LED_RED:
            leds = 1
        elif color == LED_GREEN:
            leds = 2
        elif color == LED_BLUE:
            leds = 4
        # 4 is yellow, no yellow
        elif color == LED_BLUE_GREEN:
            leds = 6
        elif color == LED_PURPLE:
            leds = 5
        elif color == LED_WHITE:
            leds = 7
        elif color == LED_RED_GREEN:
            leds = 8

        try:
            rospy.wait_for_service('niryo_robot/ttl_driver/set_dxl_leds', timeout=1)
        except rospy.ROSException:
            rospy.logwarn("Niryo ROS control LED service is not up!")
        try:
            set_dxl_leds = rospy.ServiceProxy('niryo_robot/ttl_driver/set_dxl_leds', SetInt)
            set_dxl_leds(leds)
        except rospy.ServiceException:
            rospy.logwarn("Could not call niryo_robot/ttl_driver/set_dxl_leds service")

    def set_led(self, color, dxl_leds=False):
        if self._red_led is not None:
            self._red_led.value = color & 0b100
        if self._green_led is not None:
            self._green_led.value = color & 0b010
        if self._blue_led is not None:
            self._blue_led.value = color & 0b001

        if dxl_leds:
            self.set_dxl_leds(color)

    def set_led_from_state(self, dxl_leds=False):
        if self.state == LedState.SHUTDOWN:
            self.set_led(LED_PURPLE, dxl_leds)
        elif self.state == LedState.HOTSPOT:
            self.set_led(LED_BLUE, dxl_leds)
        elif self.state == LedState.WAIT_HOTSPOT:
            self.set_led(LED_BLUE, dxl_leds)
        elif self.state == LedState.OK:
            self.set_led(LED_GREEN, dxl_leds)
        elif self.state == LedState.PAUSE:
            self.set_led(LED_RED_GREEN, dxl_leds)
        else:
            self.set_led(LED_OFF, dxl_leds)

        self._led_state_topic.publish(self.state)

    def __callback_wifi_state(self):
        if self.state == LedState.SHUTDOWN:
            return

        response = system_api_client.wifi_state()
        if not response.success:
            return

        if response.data['hotspot_state'] is True:
            if self.state != LedState.HOTSPOT:
                self.state = LedState.HOTSPOT
                self.set_led_from_state(dxl_leds=True)
        elif self.state == LedState.HOTSPOT:
            self.state = LedState.OK
            self.set_led_from_state(dxl_leds=True)

    def _callback_hardware_status(self, msg):
        # filter error 1 = input voltage
        if not msg.connection_up or list(msg.hardware_errors) > len(msg.hardware_errors) * [1]:
            self.set_led(LED_RED, dxl_leds=True)  # blink red
            rospy.sleep(0.05)
            self.set_led_from_state(dxl_leds=True)

    def _callback_set_led_state(self, req):
        state = req.value
        if state == LedState.SHUTDOWN:
            self.state = LedState.SHUTDOWN
            self.set_led_from_state(dxl_leds=True)
        elif state == LedState.WAIT_HOTSPOT:
            self.state = LedState.WAIT_HOTSPOT
            self.set_led_from_state(dxl_leds=True)
        elif state == LedState.PAUSE:
            self.state = LedState.PAUSE
            self.set_led_from_state(dxl_leds=False)
        elif state == LedState.OK:
            self.state = LedState.OK
            self.set_led_from_state(dxl_leds=False)
        else:
            return {'status': CommandStatus.LED_MANAGER_ERROR, 'message': 'Not yet implemented'}
        return {'status': CommandStatus.SUCCESS, 'message': 'Set LED OK'}

    def _callback_set_led_custom_blinker(self, req):
        self._activate_blinker = req.activate
        if self._activate_blinker:
            self._blinker_frequency = rospy.Rate(max(min(100, req.frequency), 1))
            self._blinker_color = req.color
            if req.blinker_duration > 0:
                self._blinker_stop_time = rospy.Time.now() + rospy.Duration.from_sec(req.blinker_duration)
            else:
                self._blinker_stop_time = None
            if not self._blinker_thread or not self._blinker_thread.is_alive():
                self._blinker_thread = Thread(target=self.blinker, name="led_blinker_thread")
                self._blinker_thread.start()

        return CommandStatus.SUCCESS, "Success"

    def blinker(self):
        while not rospy.is_shutdown() and self._activate_blinker:
            self.set_led(self._blinker_color)
            self._blinker_frequency.sleep()
            self.set_led_from_state()
            self._blinker_frequency.sleep()

            if self._blinker_stop_time and rospy.Time.now() >= self._blinker_stop_time:
                return
