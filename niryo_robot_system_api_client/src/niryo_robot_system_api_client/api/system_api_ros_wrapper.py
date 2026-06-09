from pydoc import locate

from niryo_robot_msgs.msg import CommandStatus

from .. import system_api_client


class SystemAPIRosWrapperException(Exception):
    pass


class SystemAPIRosWrapper(object):

    @staticmethod
    def get_setting(name):
        resp = system_api_client.get_setting(name)
        if not resp.success:
            return None
        if resp.data['type'] == 'bool':
            casted_type = resp.data[name] in ['True', 'true']
        else:
            casted_type = locate(resp.data['type'])(resp.data[name])
        return casted_type

    @staticmethod
    def set_setting(name, value):
        resp = system_api_client.set_setting(name, value)
        status = CommandStatus.SUCCESS if resp.success else CommandStatus.SYSTEM_API_CLIENT_REQUEST_FAILED
        return status, resp.detail
