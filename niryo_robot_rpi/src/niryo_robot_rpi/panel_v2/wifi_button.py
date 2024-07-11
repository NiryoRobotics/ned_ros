# wifi_button.py
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
from niryo_robot_system_api_client import system_api_client

from niryo_robot_rpi.msg import WifiButtonStatus

from ..common.rpi_ros_utils import play_connected
from .mcp_io_objects import McpIOManager


class WifiButton:

    def __init__(self, mcp_manager):
        if mcp_manager is None:
            mcp_manager = McpIOManager()

        self.__wifi_led = mcp_manager.add_led(rospy.get_param("~wifi/led_pin"), "Wifi Led")
        self.__wifi_button = mcp_manager.add_button(rospy.get_param("~wifi/button_pin"),
                                                    "Wifi Button",
                                                    pullup=True,
                                                    reverse_polarity=True)

        self.__hotspot_on = False
        self.__wifi_led.off()

        self.wifi_state_timer = rospy.Timer(rospy.Duration(1), self.__wifi_state_callback)

        self.__button_state_publisher = rospy.Publisher('/niryo_robot/wifi_button_state',
                                                        WifiButtonStatus,
                                                        queue_size=1)

        self.__wifi_button.on_press(self.on_press)
        self.__wifi_button.on_release(self.on_release)

        self.__press_mode = None
        self.__is_released = False

    def __del__(self):
        self.shutdown()

    def __wifi_state_callback(self, _):
        response = system_api_client.wifi_state()
        if not response.success:
            self.__hotspot_on = False

        if self.__hotspot_on != response.data['hotspot_state']:
            self.__hotspot_on = response.data['hotspot_state']
            if self.__hotspot_on:
                self.__wifi_led.on()
            else:
                self.__wifi_led.off()

    def __set_press_mode(self, elapsed_time):
        if elapsed_time > 10:
            self.__press_mode = WifiButtonStatus.IGNORE_PRESS
        elif elapsed_time > 7:
            self.__press_mode = WifiButtonStatus.LONG_PRESS
        elif elapsed_time > 0:
            self.__press_mode = WifiButtonStatus.SHORT_PRESS
        else:
            self.__press_mode = WifiButtonStatus.IGNORE_PRESS
            rospy.logwarn(f'User is a time traveler. elapsed time is {elapsed_time}')

    def __do_action(self):
        if self.__press_mode == WifiButtonStatus.IGNORE_PRESS:
            return
        elif self.__press_mode == WifiButtonStatus.LONG_PRESS:
            self.__reset_network()
        elif self.__press_mode == WifiButtonStatus.SHORT_PRESS:
            self.__swap_hotspot_state()

    def __swap_hotspot_state(self):
        if self.__hotspot_on:
            system_api_client.stop_hotspot()
        else:
            system_api_client.start_hotspot()

    @staticmethod
    def __reset_network():
        system_api_client.reset_wifi()
        system_api_client.reset_ethernet()
        system_api_client.reset_hotspot()
        play_connected()

    def on_press(self):
        pressed_time = rospy.Time.now()
        self.__is_released = False

        while not rospy.is_shutdown() and not self.__is_released:
            previous_press_mode = self.__press_mode
            elapsed_time = (rospy.Time.now() - pressed_time).to_sec()
            self.__set_press_mode(elapsed_time)
            if self.__press_mode != previous_press_mode:
                wifi_button_status = WifiButtonStatus()
                wifi_button_status.mode = self.__press_mode
                wifi_button_status.state = WifiButtonStatus.PRESSED
                self.__button_state_publisher.publish(wifi_button_status)

    def on_release(self):
        self.__is_released = True
        wifi_button_status = WifiButtonStatus()
        wifi_button_status.mode = self.__press_mode
        wifi_button_status.state = WifiButtonStatus.RELEASED
        self.__button_state_publisher.publish(wifi_button_status)
        self.__do_action()

    def shutdown(self):
        self.__wifi_button.disable_on_press()
        self.__wifi_button.disable_on_release()
        self.__wifi_led.off()
