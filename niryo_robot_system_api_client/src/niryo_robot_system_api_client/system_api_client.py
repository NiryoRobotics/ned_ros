import requests

from .constants import API_BASE_URL
from .SystemApiResponse import SystemApiResponse
from .SystemApiEndpoint import SystemApiEndpoint


def __request(requests_method, endpoint, *args, **kwargs):
    try:
        requests_response = requests_method(f'{API_BASE_URL}{endpoint.value}', *args, **kwargs)
    except requests.ConnectionError as connection_error:
        raise ConnectionError(str(connection_error))

    return SystemApiResponse.from_requests(requests_response)


def __get(endpoint, params=None):
    return __request(requests.get, endpoint, params=params)


def __post(endpoint, params=None):
    return __request(requests.post, endpoint, json=params)


def set_robot_name(name):
    return __post(SystemApiEndpoint.SET_ROBOT_NAME, {'name': name})


def hotspot_state():
    return __get(SystemApiEndpoint.HOTSPOT_STATE)


def wifi_state():
    return __get(SystemApiEndpoint.WIFI_STATE)


def reset_wifi():
    return __post(SystemApiEndpoint.RESET_WIFI)


def reset_hotspot():
    return __post(SystemApiEndpoint.RESET_HOTSPOT)


def reset_ethernet():
    return __post(SystemApiEndpoint.RESET_ETHERNET)


def start_hotspot():
    return __post(SystemApiEndpoint.START_HOTSPOT)


def restart_wifi():
    return __post(SystemApiEndpoint.RESTART_WIFI)


def start_wifi():
    return __post(SystemApiEndpoint.START_WIFI)


def stop_wifi():
    return __post(SystemApiEndpoint.STOP_WIFI)


def stop_hotspot():
    return __post(SystemApiEndpoint.STOP_HOTSPOT)


def ethernet_profile(profile, ip="", mask="", gateway="", dns=[]):
    return __post(SystemApiEndpoint.ETHERNET_PROFILE, {
        "profile": profile,
        "ip": ip,
        "mask": mask,
        "gw": gateway,
        "dns": dns,
    })


def set_setting(name, value):
    return __post(SystemApiEndpoint.SET_SETTING, {'name': name, 'value': value})


def get_setting(name):
    return __get(SystemApiEndpoint.GET_SETTING, {'name': name})
