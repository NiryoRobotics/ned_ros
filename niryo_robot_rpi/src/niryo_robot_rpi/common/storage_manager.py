# storage_manager.py
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

from datetime import datetime
import json
import os
import rospy
import subprocess

from niryo_robot_rpi.msg import StorageStatus, LogStatus
from niryo_robot_msgs.srv import SetInt

#
# This class will handle storage on Raspberry Pi
#


class StorageManager:

    def __init__(self):
        rospy.logdebug("StorageManager - Entering in Init")
        self.log_size_threshold = rospy.get_param("~ros_log_size_threshold")
        self.log_path = rospy.get_param("~ros_log_location")
        self.should_purge_log_on_startup_file = rospy.get_param("~should_purge_ros_log_on_startup_file")
        self.purge_log_on_startup = self.should_purge_log_on_startup()

        self.__run_ids_filename = 'run_ids.json'
        self.__update_run_ids_file()

        # clean log on startup if param is true
        if self.purge_log_on_startup:
            rospy.logwarn("Purging ROS log on startup !")
            self.purge_log()

        self.purge_log_server = rospy.Service('/niryo_robot_rpi/purge_ros_logs', SetInt, self.callback_purge_log)

        self.change_purge_log_on_startup_server = rospy.Service('/niryo_robot_rpi/set_purge_ros_log_on_startup',
                                                                SetInt,
                                                                self.callback_change_purge_log_on_startup)

        self.storage_status_publisher = rospy.Publisher('/niryo_robot_rpi/storage_status', StorageStatus, queue_size=10)
        # TODO: delete in 5.1
        self.log_status_publisher = rospy.Publisher('/niryo_robot_rpi/ros_log_status', LogStatus, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(3), self.publish_storage_status)

        rospy.loginfo("Init Storage Manager OK")

    def __update_run_ids_file(self):
        with open(f'{self.log_path}/{self.__run_ids_filename}', 'a+') as file:
            try:
                run_ids = json.load(file)
            except json.JSONDecodeError:
                run_ids = {}
            run_ids[datetime.now().isoformat()] = rospy.get_param('/run_id')
            file.seek(0)
            file.truncate()
            json.dump(run_ids, file, indent=2)

    def __clear_run_ids_file(self):
        with open(f'{self.log_path}/{self.__run_ids_filename}', 'w') as file:
            run_ids = {}
            json.dump(run_ids, file, indent=2)

    @staticmethod
    def get_storage(value):
        try:
            process = subprocess.Popen(['df', f'--output={value}', '/'], stdout=subprocess.PIPE)
            output, error = process.communicate()
            lines = output.decode().split(os.linesep)
            if len(lines) >= 2:
                return int(int(lines[1]) / 1024)
            return -1
        except subprocess.CalledProcessError:
            return -1

    def get_available_disk_size(self):
        return self.get_storage('avail')

    def get_total_disk_size(self):
        return self.get_storage('size')

    def get_log_size(self):
        try:
            if not os.path.isdir(self.log_path):
                return -1
            output = subprocess.check_output(['du', '-sBM', self.log_path])
            output_array = output.decode().split()
            if len(output_array) >= 1:
                return int(output_array[0].replace('M', ''))
            return -1
        except subprocess.CalledProcessError:
            return -1

    # !! ros logs after using this method will not be saved
    # need to restart ros completly to save new logs
    def purge_log(self):
        try:
            subprocess.call(['rosclean', 'purge', '--size', '50', '-y'])
            self.__clear_run_ids_file()
            rospy.loginfo("ROS Log Manager - Purging done")
            return True
        except subprocess.CalledProcessError as e:
            rospy.logwarn(e)
            return False

    def should_purge_log_on_startup(self):
        if os.path.isfile(self.should_purge_log_on_startup_file):
            with open(self.should_purge_log_on_startup_file, 'r') as f:
                for line in f:
                    if not (line.startswith('#') or len(line) == 0):
                        condition = line.rstrip()
                        if condition == "true":
                            return True
                        return False
        return False

    def change_purge_log_on_startup(self, condition):
        with open(self.should_purge_log_on_startup_file, 'w') as f:
            value = ""
            if condition:
                value = "true"
            else:
                value = "false"
            f.write(value)

        # After writing, read new value from file
        self.purge_log_on_startup = self.should_purge_log_on_startup()

    #
    # ----- ROS Interface below -----
    #

    @staticmethod
    def create_response(status, message):
        return {'status': status, 'message': message}

    def callback_purge_log(self, req):
        rospy.logwarn("Purge ROS logs on user request")
        if self.purge_log():
            return self.create_response(
                200,
                "ROS logs have been purged. " + "Following logs will be discarded. If you want to get logs, you " +
                "need to restart the robot")
        return self.create_response(400, "Unable to remove ROS logs")

    def callback_change_purge_log_on_startup(self, req):
        if req.value == 1:
            self.change_purge_log_on_startup(True)
        else:
            self.change_purge_log_on_startup(False)
        return self.create_response(200, "Purge log on startup value has been changed")

    def publish_storage_status(self, event):
        msg = StorageStatus()
        msg.header.stamp = rospy.Time.now()
        msg.log_size = self.get_log_size()
        msg.available_disk_size = self.get_available_disk_size()
        msg.total_disk_size = self.get_total_disk_size()
        msg.purge_log_on_startup = self.purge_log_on_startup
        self.storage_status_publisher.publish(msg)

        # TODO: delete in 5.1
        msg = LogStatus()
        msg.header.stamp = rospy.Time.now()
        msg.log_size = self.get_log_size()
        msg.available_disk_size = self.get_available_disk_size()
        msg.purge_log_on_startup = self.purge_log_on_startup
        self.log_status_publisher.publish(msg)
