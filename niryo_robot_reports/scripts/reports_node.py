#!/usr/bin/env python3
import threading
from tempfile import TemporaryDirectory
# Libs
import os
import rospy
from distutils.dir_util import mkpath

from niryo_robot_reports.msg import Service
from niryo_robot_system_api_client import system_api_client
from niryo_robot_system_api_client.msg import Setting
from niryo_robot_utils import sentry_init, async_init

from niryo_robot_reports.CloudAPI import CloudAPI
from niryo_robot_reports.AlertReportHandler import AlertReportHandler
from niryo_robot_reports.DailyReportHandler import DailyReportHandler
from niryo_robot_reports.TestReportHandler import TestReportHandler
from niryo_robot_reports.AutoDiagnosisReportHandler import AutoDiagnosisReportHandler

# msg
from niryo_robot_msgs.msg import CommandStatus

# srv
from niryo_robot_reports.srv import CheckConnection


class ReportsNode:

    def __init__(self):
        rospy.logdebug("Reports Node - Entering in Init")

        self.__lazy_loaded_cloud_api = None

        threading.Thread(target=self.__on_get_settings_available).start()

        rospy.Service("~check_connection", CheckConnection, self.__check_connection_callback)

        rospy.Subscriber("/niryo_robot_system_api_client/setting_update", Setting, self.__setting_update_callback)

        rospy.logdebug("Reports Node - Node Started")

    @property
    def __cloud_api(self):
        if self.__lazy_loaded_cloud_api is None:
            raise RuntimeError("Cloud API is not initialized yet")
        return self.__lazy_loaded_cloud_api

    def __on_get_settings_available(self):
        try:
            system_api_client.wait_for_api(rospy.is_shutdown, timeout=60)
        except TimeoutError:
            rospy.logfatal("System API is not available. Reports node will not be able to start.")
            return

        settings = {}
        for setting in ["serial_number", "rasp_id", "api_key", "sharing_allowed"]:
            response = system_api_client.get_setting(setting)
            if not response.success:
                rospy.logerr(f'Unable to get setting "{setting}"')
                settings[setting] = None
                continue
            settings[setting] = response.data[setting]

        settings["cloud_domain"] = os.getenv("NED_ROS_CLOUD_DOMAIN")
        if settings["cloud_domain"] is None:
            raise EnvironmentError("NED_ROS_CLOUD_DOMAIN is not set")

        self.__lazy_loaded_cloud_api = CloudAPI(**settings, https=True)

        response = system_api_client.get_setting("reports_path")
        if response.success:
            reports_path = os.path.expanduser(response.data["reports_path"])
            if not os.path.isdir(reports_path):
                mkpath(reports_path)
        else:
            reports_path = TemporaryDirectory().name
            rospy.logerr("Unable to retrieve the reports directory path from the database")

        DailyReportHandler(self.__cloud_api, reports_path)
        TestReportHandler(self.__cloud_api, reports_path)
        AlertReportHandler(self.__cloud_api)
        AutoDiagnosisReportHandler(self.__cloud_api)

        # Set a bool to mention that this node is initialized
        rospy.set_param("~initialized", True)

    def __check_connection_callback(self, req):
        rospy.logdebug("service called: " + str(req.service.to_test))

        try:
            {
                Service.TEST_REPORTS: self.__cloud_api.test_reports.ping,
                Service.DAILY_REPORTS: self.__cloud_api.daily_reports.ping,
                Service.ALERT_REPORTS: self.__cloud_api.alert_reports.ping,
                Service.AUTO_DIAGNOSIS_REPORTS: self.__cloud_api.auto_diagnosis_reports.ping,
            }[req.service.to_test]()
        except KeyError:
            return CommandStatus.REPORTS_SERVICE_UNREACHABLE, False

        return CommandStatus.SUCCESS, True

    def __setting_update_callback(self, req):
        try:
            {
                "serial_number": self.__cloud_api.set_serial_number,
                "api_key": self.__cloud_api.set_api_key,
                "sharing_allowed": lambda v: self.__cloud_api.set_sharing_allowed(v == "True"),
                "rasp_id": self.__cloud_api.set_rasp_id,
            }[req.name](req.value)
        except KeyError:
            pass
        except RuntimeError as e:
            rospy.logwarn(f'Received update for setting "{req.name}" but: {e}')
            pass


if __name__ == "__main__":
    sentry_init()

    rospy.init_node("niryo_robot_reports", anonymous=False, log_level=rospy.INFO)

    try:
        node = ReportsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
