#!/usr/bin/env python

# Libs
import rospy
from niryo_robot_utils import sentry_init

from niryo_robot_system_api_client import system_api_client

# msg
from niryo_robot_msgs.msg import SoftwareVersion, CommandStatus
from niryo_robot_system_api_client.msg import Setting as SettingMsg
from niryo_robot_database.msg import Setting as DbSettingMsg
# srv
from niryo_robot_system_api_client.srv import (SetSettings,
                                               SetSettingsRequest,
                                               SetSettingsResponse,
                                               GetSettings,
                                               GetSettingsRequest,
                                               GetSettingsResponse)


def _get_setting_callback(req: GetSettingsRequest) -> GetSettingsResponse:
    resp = system_api_client.get_setting(req.name)
    if not resp.success:
        return GetSettingsResponse(status=CommandStatus.SYSTEM_API_CLIENT_REQUEST_FAILED, value=resp.detail)
    return GetSettingsResponse(status=CommandStatus.SUCCESS, type=resp.data['type'], value=resp.data[req.name])


def _set_setting_callback(req: SetSettingsRequest) -> SetSettingsResponse:
    resp = system_api_client.set_setting(req.name, req.value)
    status = CommandStatus.SUCCESS if resp.success else CommandStatus.SYSTEM_API_CLIENT_REQUEST_FAILED
    return SetSettingsResponse(status=status, message=resp.detail)


def _declare_aliases():
    # aliases for retro compatibility
    rospy.Service('/niryo_robot_database/settings/get', GetSettings, _get_setting_callback)
    rospy.Service('/niryo_robot_database/settings/set', SetSettings, _set_setting_callback)

    pub = rospy.Publisher('/niryo_robot_database/setting_update', DbSettingMsg, queue_size=5)
    rospy.Subscriber('~setting_update', SettingMsg, lambda x: pub.publish(x), queue_size=5)


class SystemApiNode:

    def __init__(self):
        rospy.logdebug("System API Node - Entering in Init")

        self.__sw_version_subscriber = rospy.Subscriber('/niryo_robot_hardware_interface/software_version',
                                                        SoftwareVersion,
                                                        self.__sw_callback,
                                                        queue_size=1)

        # proxys for CPP nodes
        rospy.Service('~settings/get', GetSettings, _get_setting_callback)
        rospy.Service('~settings/set', SetSettings, _set_setting_callback)

        _declare_aliases()

        # Set a bool to mentioned this node is initialized
        rospy.set_param('~initialized', True)

        rospy.logdebug("System API Node - Node Started")

    def __sw_callback(self, msg):
        motors_names = ['motor_1', 'motor_2', 'motor_3', 'motor_4', 'motor_5', 'motor_6', 'end_effector']
        for (motor_name, motor_version, model_number) in zip(motors_names,
                                                             msg.stepper_firmware_versions,
                                                             msg.model_numbers):
            system_api_client.set_version(motor_name, motor_version)
            system_api_client.set_version(motor_name + "_mn", str(model_number))

        self.__sw_version_subscriber.unregister()


if __name__ == "__main__":
    sentry_init()

    rospy.init_node('niryo_robot_system_api_client', anonymous=False, log_level=rospy.INFO)
    try:
        node = SystemApiNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
