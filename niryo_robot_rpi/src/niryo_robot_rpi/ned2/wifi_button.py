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
from threading import Lock
from niryo_robot_system_api_client.HttpClient import HttpClient as SystemApiClient

from .mcp_io_objects import McpIOManager

from niryo_robot_rpi.common.rpi_ros_utils import play_connected

from niryo_robot_system_api_client.msg import WifiStatus


class WifiButton:

    def __init__(self, mcp_manager, is_top_button_pressed):

        self.__system_api_client = SystemApiClient()

        self.__mcp_manager = mcp_manager if mcp_manager is not None else McpIOManager()

        self.__wifi_led = mcp_manager.add_led(rospy.get_param("~wifi/led_pin"), "Wifi Led")
        self.__wifi_button = mcp_manager.add_button(rospy.get_param("~wifi/button_pin"),
                                                    "Wifi Button",
                                                    pullup=True,
                                                    reverse_polarity=True)
        self.__is_top_button_pressed = is_top_button_pressed

        self.__timer = None
        self.__set_hotspot_lock = Lock()
        self.__wlan0_status = WifiStatus.UNKNOWN
        self.__hotspot_status = WifiStatus.UNKNOWN
        self.__wifi_led.off()

        self.__wifi_status_sub = rospy.Subscriber('/niryo_robot/wifi/status', WifiStatus, self.__wifi_status_callback)

        self.__wifi_button.on_press(self.on_press)
        self.__wifi_button.on_release(self.on_release)

        self.__pressed_time = None
        self.__released_time = None

    def __del__(self):
        self.shutdown()

    def __wifi_status_callback(self, msg):
        if self.__hotspot_status != msg.hotspot_status or self.__wlan0_status != msg.wlan0_status:
            self.__hotspot_status = msg.hotspot_status
            self.__wlan0_status = msg.wlan0_status
            self.set_led_behaviour()

    def set_led_behaviour(self):
        if self.__wlan0_status == self.__hotspot_status == WifiStatus.OFF:
            # no wifi
            self.stop_blink()
            self.__wifi_led.off()
        elif self.__wlan0_status == WifiStatus.OFF and self.__hotspot_status == WifiStatus.ON:
            # hotspot mode
            self.stop_blink()
            self.__wifi_led.on()
        elif self.__wlan0_status == WifiStatus.ON:
            if self.__timer is not None:
                self.__timer.shutdown()
            self.__timer = rospy.Timer(rospy.Duration.from_sec(2), self.__bink_pulse_wifi_led)

    def stop_blink(self):
        if self.__timer is not None:
            self.__timer.shutdown()
            self.__timer = None

    def __bink_wifi_led(self, _):
        self.__wifi_led.reverse_state()

    def __bink_pulse_wifi_led(self, _):
        self.__wifi_led.on()
        rospy.sleep(0.2)
        self.__wifi_led.off()
        rospy.sleep(0.2)
        self.__wifi_led.on()
        rospy.sleep(0.2)
        self.__wifi_led.off()

    def on_press(self):
        """
        long press (>= 2s)
        """
        self.__pressed_time = rospy.Time.now()
        self.__released_time = None

        while not rospy.is_shutdown() and self.__released_time is None:
            if (rospy.Time.now() - self.__pressed_time).to_sec() >= 2:
                if self.__is_top_button_pressed():
                    self.__system_api_client.reset_wifi()
                    self.__system_api_client.reset_hotspot()
                    self.__system_api_client.reset_ethernet()
                    play_connected()
                else:
                    if self.__hotspot_status == WifiStatus.ON:
                        self.__system_api_client.stop_wifi()
                        self.__system_api_client.stop_hotspot()
                    else:
                        self.__system_api_client.start_wifi()
                        self.__system_api_client.start_hotspot()
                # self.__wifi_led.off()
                return
            rospy.sleep(0.1)

    def on_release(self):
        """
        short press (<2s)
        """
        self.__released_time = rospy.Time.now()
        if (self.__released_time - self.__pressed_time).to_sec() < 2 and not self.__set_hotspot_lock.locked():
            with self.__set_hotspot_lock:
                if self.__wlan0_status == WifiStatus.ON:
                    self.__system_api_client.stop_wifi()
                else:
                    self.__system_api_client.start_wifi()

    def shutdown(self):
        self.stop_blink()
        self.__wifi_button.disable_on_press()
        self.__wifi_button.disable_on_release()
        self.__wifi_led.off()
