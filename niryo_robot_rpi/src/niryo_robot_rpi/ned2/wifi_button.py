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
from niryo_robot_system_api_client.HttpClient import HttpClient as SystemApiClient

from .mcp_io_objects import McpIOManager

from niryo_robot_rpi.common.rpi_ros_utils import play_connected

from niryo_robot_system_api_client.msg import WifiStatus
from niryo_robot_rpi.msg import WifiButtonStatus


class WifiButton:

    def __init__(self, mcp_manager):

        self.__system_api_client = SystemApiClient()

        if mcp_manager is None:
            mcp_manager = McpIOManager()

        self.__wifi_led = mcp_manager.add_led(rospy.get_param("~wifi/led_pin"), "Wifi Led")
        self.__wifi_button = mcp_manager.add_button(rospy.get_param("~wifi/button_pin"),
                                                    "Wifi Button",
                                                    pullup=True,
                                                    reverse_polarity=True)

        self.__timer = None
        self.__wlan0_status = WifiStatus.UNKNOWN
        self.__hotspot_status = WifiStatus.UNKNOWN
        self.__wifi_led.off()

        rospy.Subscriber('/niryo_robot/wifi/status', WifiStatus, self.__wifi_status_callback)

        self.__button_state_publisher = rospy.Publisher('/niryo_robot/wifi_button_state',
                                                        WifiButtonStatus,
                                                        queue_size=1)

        self.__wifi_button.on_press(self.on_press)
        self.__wifi_button.on_release(self.on_release)

        self.__press_mode = None
        self.__is_released = False

    def __del__(self):
        self.shutdown()

    def __wifi_status_callback(self, msg):
        if self.__hotspot_status != msg.hotspot_status or self.__wlan0_status != msg.wlan0_status:
            self.__hotspot_status = msg.hotspot_status
            self.__wlan0_status = msg.wlan0_status
            self.set_led_behaviour()

    def set_led_behaviour(self):
        if self.__wlan0_status == WifiStatus.OFF and self.__hotspot_status == WifiStatus.OFF:
            self.led_off()
        elif self.__wlan0_status == WifiStatus.ON and self.__hotspot_status == WifiStatus.OFF:
            self.led_blink(self.irregular_blink_pattern)
        elif self.__wlan0_status == WifiStatus.OFF and self.__hotspot_status == WifiStatus.ON:
            self.led_blink(self.regular_blink_pattern)
        else:
            self.led_on()

    def stop_blink(self):
        if self.__timer is not None:
            self.__timer.shutdown()
            self.__timer = None

    def led_off(self):
        self.stop_blink()
        self.__wifi_led.off()

    def led_on(self):
        self.stop_blink()
        self.__wifi_led.on()

    def led_blink(self, blink_pattern):
        self.stop_blink()
        self.__timer = rospy.Timer(rospy.Duration.from_sec(1), blink_pattern)

    def irregular_blink_pattern(self, _):
        self.__wifi_led.on()
        rospy.sleep(0.2)
        self.__wifi_led.off()
        rospy.sleep(0.2)
        self.__wifi_led.on()
        rospy.sleep(0.2)
        self.__wifi_led.off()
        rospy.sleep(0.4)

    def regular_blink_pattern(self, _):
        self.__wifi_led.on()
        rospy.sleep(0.5)
        self.__wifi_led.off()
        rospy.sleep(0.5)

    def __set_press_mode(self, elapsed_time):
        if elapsed_time > 10:
            self.__press_mode = WifiButtonStatus.IGNORE_PRESS
        elif elapsed_time > 7:
            self.__press_mode = WifiButtonStatus.VERY_LONG_PRESS
        elif elapsed_time > 2:
            self.__press_mode = WifiButtonStatus.LONG_PRESS
        elif elapsed_time > 0:
            self.__press_mode = WifiButtonStatus.SHORT_PRESS
        else:
            rospy.logwarn(f'User is a time traveler. elapsed time is {elapsed_time}')

    def __do_action(self):
        fun = {
            WifiButtonStatus.SHORT_PRESS: self.__swap_hotspot_state,
            WifiButtonStatus.LONG_PRESS: self.__swap_wifi_state,
            WifiButtonStatus.VERY_LONG_PRESS: self.__reset_network,
            WifiButtonStatus.IGNORE_PRESS: lambda: None,
        }[self.__press_mode]
        fun()

    def __swap_wifi_state(self):
        if self.__wlan0_status == WifiStatus.ON:
            self.__system_api_client.stop_wifi()
        else:
            self.__system_api_client.start_wifi()

    def __swap_hotspot_state(self):
        if self.__hotspot_status == WifiStatus.ON:
            self.__system_api_client.stop_hotspot()
        else:
            self.__system_api_client.start_hotspot()

    def __reset_network(self):
        self.__system_api_client.reset_wifi()
        self.__system_api_client.reset_ethernet()
        self.__system_api_client.reset_hotspot()
        play_connected()

    def on_press(self):
        pressed_time = rospy.Time.now()
        self.__is_released = False

        while not rospy.is_shutdown() and not self.__is_released:
            press_mode = self.__press_mode
            elapsed_time = (rospy.Time.now() - pressed_time).to_sec()
            self.__set_press_mode(elapsed_time)
            if self.__press_mode != press_mode:
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
        self.stop_blink()
        self.__wifi_button.disable_on_press()
        self.__wifi_button.disable_on_release()
        self.__wifi_led.off()
