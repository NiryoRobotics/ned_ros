# Lib
import math
import subprocess
import rospy
import os
import numpy as np

# Messages
from std_msgs.msg import UInt8

# Services
from niryo_robot_msgs.srv import SetInt

# Command Status
from niryo_robot_msgs.msg import CommandStatus


class VolumeManager:
    def __init__(self):
        # - Init
        self.__simulation_mode = rospy.get_param("~simulation_mode")
        self.__volume_file_path = os.path.expanduser(rospy.get_param(
            "~volume_file_path_simulation" if self.__simulation_mode else "~volume_file_path"))

        self.__volume_percentage = None
        self.__max_volume = rospy.get_param("~max_volume")
        self.__min_volume = rospy.get_param("~min_volume")
        self.__volume_factor = rospy.get_param("~volume_factor")

        # - Publisher
        self.__volume_state_publisher = rospy.Publisher('/niryo_robot_sound/volume', UInt8, latch=True, queue_size=10)

        # - Init volume
        success, volume = self.__read_volume_file()
        if success:
            self.set_volume(volume)
        else:
            self.set_volume(
                rospy.get_param("~default_volume_simulation" if self.__simulation_mode else "~default_volume"))

        # - Services
        rospy.Service('/niryo_robot_sound/set_volume', SetInt, self.__callback_set_volume)

    # - Callbacks
    def __callback_set_volume(self, msg):
        self.set_volume(msg.value)
        return CommandStatus.SUCCESS, "Volume set at {}%".format(self.__volume_percentage)

    # - Main class usage
    @property
    def volume(self):
        # Out oft simulation the volume is managed by the rpi
        if not self.__simulation_mode:
            return 100

        # In simulation the volume is managed by ffplay
        return self.__volume_percentage

    @property
    def raw_volume(self):
        return self.__volume_percentage

    def set_volume(self, percentage):
        if percentage != self.__volume_percentage:
            self.__volume_percentage = self.__check_before_set_volume(percentage)
            self.__set_robot_volume()
            self.__volume_state_publisher.publish(self.__volume_percentage)
            self.__write_volume_file()

    def fade_out(self, duration=2.5):
        volume_list = np.linspace(self.__volume_percentage, 0, 12)[1:-1]
        sleep_duration = rospy.Duration.from_sec(duration / len(volume_list))
        for vol in volume_list:
            loop_time = rospy.Time.now()
            self.__volume_percentage = vol
            self.__set_robot_volume()
            rospy.sleep((loop_time + sleep_duration) - rospy.Time.now())

    def __set_robot_volume(self):
        if not self.__simulation_mode:
            volume_decibels = self.__volume_to_decibels(self.__volume_percentage * self.__volume_factor)
            # args = ("amixer", "-q", "set", "Speaker", "--", str(volume_decibels) + "dB")
            args = ("amixer", "set", "Speaker", "--", str(volume_decibels) + "dB")
            p = subprocess.check_output(args)

    @staticmethod
    def __volume_to_decibels(volume_percentage):
        if volume_percentage <= 0:
            return -100000
        else:
            return round(20 * math.log10(volume_percentage / 100.), 2)

    def __check_before_set_volume(self, volume):
        """
        Check if the volume set by the user is correct.
        Volume should be between 0 and 100.

        :return: A float corresponding to the volume
        :rtype: float
        """
        return max(self.__min_volume, min(self.__max_volume, volume))

    def __read_volume_file(self):
        if os.path.isfile(self.__volume_file_path):
            with open(self.__volume_file_path, 'r') as f:
                for line in f:
                    if not (line.startswith('#') or len(line) == 0):
                        try:
                            volume = int(line.rstrip())
                        except ValueError:
                            return False, 0
                        return True, volume
        return False, 0

    def __write_volume_file(self):
        try:
            with open(self.__volume_file_path, 'w') as f:
                comment = "# THIS IS A GENERATED FILE\n"
                comment += "# Here is the sound volume of the robot\n"
                f.write(comment)
                f.write(str(self.__volume_percentage))
                return True
        except EnvironmentError as e:
            print(e)
            return False
