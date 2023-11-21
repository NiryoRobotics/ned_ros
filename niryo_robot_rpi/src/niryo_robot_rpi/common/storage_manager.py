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
from pathlib import Path
import rospy
import subprocess

from niryo_robot_rpi.msg import StorageStatus, LogStatus
from niryo_robot_msgs.srv import SetInt

from niryo_robot_database.srv import GetSettings, SetSettings

#
# This class will handle storage on Raspberry Pi
#


class StorageManager:

    def __init__(self):
        rospy.logdebug("StorageManager - Entering in Init")

        process_result = subprocess.run(['roslaunch-logs'], capture_output=True, encoding='utf-8', check=True)
        current_ros_logs_path = Path(process_result.stdout.strip())
        self.__log_path = current_ros_logs_path.parent
        self.__run_id = current_ros_logs_path.stem

        get_setting_service = rospy.ServiceProxy('/niryo_robot_database/settings/get', GetSettings)
        get_setting_response = get_setting_service('purge_ros_logs_on_startup')
        self.__purge_log_on_startup = get_setting_response.value == 'True'

        self.__run_ids_file = self.__log_path.joinpath('run_ids.json')

        # clean log on startup if param is true
        if self.__purge_log_on_startup:
            rospy.logwarn("Purging ROS log on startup !")
            self.purge_log()

        self.__update_run_ids_file()

        self.purge_log_server = rospy.Service('/niryo_robot_rpi/purge_ros_logs', SetInt, self.callback_purge_log)

        self.change_purge_log_on_startup_server = rospy.Service('/niryo_robot_rpi/set_purge_ros_log_on_startup',
                                                                SetInt,
                                                                self.callback_change_purge_log_on_startup)

        self.storage_status_publisher = rospy.Publisher('/niryo_robot_rpi/storage_status', StorageStatus, queue_size=10)

        # NS1 only
        self.log_status_publisher = rospy.Publisher('/niryo_robot_rpi/ros_log_status', LogStatus, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(3), self.publish_storage_status)

        rospy.loginfo("Init Storage Manager OK")

    def __update_run_ids_file(self):
        try:
            run_ids = json.loads(self.__run_ids_file.read_text())
        except (OSError, json.JSONDecodeError):
            run_ids = {}
        run_ids[datetime.now().isoformat()] = self.__run_id
        self.__run_ids_file.write_text(json.dumps(run_ids, indent=2))

    def __clear_run_ids_file(self):
        self.__run_ids_file.write_text('{}')

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
            if not self.__log_path.is_dir():
                return -1
            process_result = subprocess.run(['du', '-sBM', str(self.__log_path)], capture_output=True, encoding='utf-8')
            output_array = process_result.stdout.split()
            if len(output_array) >= 1:
                return int(output_array[0].replace('M', ''))
            return -1
        except subprocess.CalledProcessError:
            return -1

    def purge_log(self):
        try:
            subprocess.call(['rosclean', 'purge', '--size', '50', '-y'])
            self.__clear_run_ids_file()
            rospy.loginfo("ROS Log Manager - Purging done")
            return True
        except subprocess.CalledProcessError as e:
            rospy.logwarn(e)
            return False

    #
    # ----- ROS Interface below -----
    #

    @staticmethod
    def create_response(status, message):
        return {'status': status, 'message': message}

    def callback_purge_log(self, _):
        rospy.logwarn("Purge ROS logs on user request")
        if self.purge_log():
            return self.create_response(
                200,
                "ROS logs have been purged. " + "Following logs will be discarded. If you want to get logs, you " +
                "need to restart the robot")
        return self.create_response(400, "Unable to remove ROS logs")

    def callback_change_purge_log_on_startup(self, req):
        set_setting_service = rospy.ServiceProxy('/niryo_robot_database/settings/set', SetSettings)
        set_setting_service(name='purge_ros_logs', value=str(req.value == 1), type='bool')
        return self.create_response(200, "Purge log on startup value has been changed")

    def publish_storage_status(self, _):
        msg = StorageStatus()
        msg.header.stamp = rospy.Time.now()
        msg.log_size = self.get_log_size()
        msg.available_disk_size = self.get_available_disk_size()
        msg.total_disk_size = self.get_total_disk_size()
        msg.purge_log_on_startup = self.__purge_log_on_startup
        self.storage_status_publisher.publish(msg)

        # NS1 only
        msg = LogStatus()
        msg.header.stamp = rospy.Time.now()
        msg.log_size = self.get_log_size()
        msg.available_disk_size = self.get_available_disk_size()
        msg.purge_log_on_startup = self.__purge_log_on_startup
        self.log_status_publisher.publish(msg)
