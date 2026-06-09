from typing import Callable

import requests
import time

from .SystemApiEndpoint import SystemApiEndpoint
from .SystemApiResponse import SystemApiResponse
from .constants import API_BASE_URL


def wait_for_api(hard_stop: Callable[[], bool] = None, timeout: int = 5):
    if hard_stop is None:

        def hard_stop():
            return False

    deadline = time.monotonic() + timeout
    while not (hs := hard_stop()) and time.monotonic() < deadline:
        if root().success:
            return
        time.sleep(0.2)
    raise TimeoutError(f'API did not respond in time. {timeout=}, hard_stop={hs}')


def __request(method, endpoint, params=None, body_params=None):
    try:
        requests_response = requests.request(method, f'{API_BASE_URL}{endpoint.value}', params=params, json=body_params)
    except requests.ConnectionError as connection_error:
        return SystemApiResponse(-1, None, str(connection_error), None)

    return SystemApiResponse.from_requests(requests_response)


def root() -> SystemApiResponse:
    return __request('GET', SystemApiEndpoint.ROOT)


def set_robot_name(name) -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.SET_ROBOT_NAME, body_params={'name': name})


def hotspot_state() -> SystemApiResponse:
    return __request('GET', SystemApiEndpoint.HOTSPOT_STATE)


def wifi_state() -> SystemApiResponse:
    return __request('GET', SystemApiEndpoint.WIFI_STATE)


def reset_wifi() -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.RESET_WIFI)


def reset_hotspot() -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.RESET_HOTSPOT)


def reset_ethernet() -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.RESET_ETHERNET)


def start_hotspot() -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.START_HOTSPOT)


def restart_wifi() -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.RESTART_WIFI)


def start_wifi() -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.START_WIFI)


def stop_wifi() -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.STOP_WIFI)


def stop_hotspot() -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.STOP_HOTSPOT)


def ethernet_profile(profile, ip="", mask="", gateway="", dns=[]) -> SystemApiResponse:
    return __request('POST',
                     SystemApiEndpoint.ETHERNET_PROFILE,
                     body_params={
                         "profile": profile,
                         "ip": ip,
                         "mask": mask,
                         "gw": gateway,
                         "dns": dns,
                     })


def set_setting(name, value) -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.SET_SETTING, body_params={'name': name, 'value': value})


def get_setting(name, with_type=True) -> SystemApiResponse:
    return __request('GET', SystemApiEndpoint.GET_SETTING, {'name': name, 'with_type': with_type})


def get_system_version_current() -> SystemApiResponse:
    return __request('GET', SystemApiEndpoint.SYSTEM_VERSION_CURRENT)


def get_version(name) -> SystemApiResponse:
    return __request('GET', SystemApiEndpoint.GET_VERSION, {'name': name})


def set_version(name, version) -> SystemApiResponse:
    return __request('POST', SystemApiEndpoint.SET_VERSION, body_params={'name': name, 'version': version})


def get_programs() -> SystemApiResponse:
    return __request('GET', SystemApiEndpoint.GET_PROGRAMS)


def get_program(id_: str) -> SystemApiResponse:
    return __request('GET', SystemApiEndpoint.GET_PROGRAM, params={'id': id_})


def program_exists(id_: str) -> SystemApiResponse:
    return __request('GET', SystemApiEndpoint.PROGRAM_EXISTS, params={'id': id_})


def add_program(id_: str, name: str, description: str, has_blockly: bool) -> SystemApiResponse:
    return __request('POST',
                     SystemApiEndpoint.ADD_PROGRAM,
                     body_params={
                         'id': id_,
                         'name': name,
                         'description': description,
                         'has_blockly': has_blockly,
                     })


def update_program(id_: str, name: str = None, description: str = None, has_blockly: bool = None) -> SystemApiResponse:
    body = {'id': id_}
    if name is not None:
        body['name'] = name
    if description is not None:
        body['description'] = description
    if has_blockly is not None:
        body['has_blockly'] = has_blockly
    return __request('POST', SystemApiEndpoint.UPDATE_PROGRAM, body_params=body)


def delete_program(id_: str) -> SystemApiResponse:
    return __request('DELETE', SystemApiEndpoint.DELETE_PROGRAM, params={'id': id_})


def get_file_paths(f_type: str = None) -> SystemApiResponse:
    params = {'f_type': f_type} if f_type is not None else None
    return __request('GET', SystemApiEndpoint.GET_FILE_PATHS, params=params)


def add_file_path(f_type: str, name: str, path: str) -> SystemApiResponse:
    return __request('POST',
                     SystemApiEndpoint.ADD_FILE_PATH,
                     body_params={
                         'f_type': f_type,
                         'name': name,
                         'path': path,
                     })


def rm_file_path(_id: str = None, name: str = None) -> SystemApiResponse:
    params = {}
    if _id is not None:
        params['id'] = _id
    if name is not None:
        params['name'] = name
    return __request('DELETE', SystemApiEndpoint.RM_FILE_PATH, params=params)
