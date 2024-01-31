import requests

from .constants import API_BASE_URL
from .SystemApiResponse import SystemApiResponse
from .SystemApiEndpoint import SystemApiEndpoint


def __request(method, endpoint, params=None, body_params=None):
    try:
        requests_response = requests.request(method, f'{API_BASE_URL}{endpoint.value}', params=params, json=body_params)
    except requests.ConnectionError as connection_error:
        return SystemApiResponse(-1, None, str(connection_error), None)

    return SystemApiResponse.from_requests(requests_response)


def set_robot_name(name):
    return __request('POST', SystemApiEndpoint.SET_ROBOT_NAME, body_params={'name': name})


def hotspot_state():
    return __request('GET', SystemApiEndpoint.HOTSPOT_STATE)


def wifi_state():
    return __request('GET', SystemApiEndpoint.WIFI_STATE)


def reset_wifi():
    return __request('POST', SystemApiEndpoint.RESET_WIFI)


def reset_hotspot():
    return __request('POST', SystemApiEndpoint.RESET_HOTSPOT)


def reset_ethernet():
    return __request('POST', SystemApiEndpoint.RESET_ETHERNET)


def start_hotspot():
    return __request('POST', SystemApiEndpoint.START_HOTSPOT)


def restart_wifi():
    return __request('POST', SystemApiEndpoint.RESTART_WIFI)


def start_wifi():
    return __request('POST', SystemApiEndpoint.START_WIFI)


def stop_wifi():
    return __request('POST', SystemApiEndpoint.STOP_WIFI)


def stop_hotspot():
    return __request('POST', SystemApiEndpoint.STOP_HOTSPOT)


def ethernet_profile(profile, ip="", mask="", gateway="", dns=[]):
    return __request('POST',
                     SystemApiEndpoint.ETHERNET_PROFILE,
                     body_params={
                         "profile": profile,
                         "ip": ip,
                         "mask": mask,
                         "gw": gateway,
                         "dns": dns,
                     })


def set_setting(name, value):
    return __request('POST', SystemApiEndpoint.SET_SETTING, body_params={'name': name, 'value': value})


def get_setting(name, with_type=True):
    return __request('GET', SystemApiEndpoint.GET_SETTING, {'name': name, 'with_type': with_type})


def get_system_version_current():
    return __request('GET', SystemApiEndpoint.SYSTEM_VERSION_CURRENT)
