from enum import Enum


class SystemApiEndpoint(Enum):
    SET_ROBOT_NAME = '/setRobotName'
    HOTSPOT_STATE = '/hotspotState'
    WIFI_STATE = '/wifiState'
    RESET_WIFI = '/resetWifi'
    RESET_HOTSPOT = '/resetHotspot'
    RESET_ETHERNET = '/resetEthernet'
    START_HOTSPOT = '/startHotspot'
    RESTART_WIFI = '/restartWifi'
    START_WIFI = '/startWifi'
    STOP_WIFI = '/stopWifi'
    STOP_HOTSPOT = '/stopHotspot'
    ETHERNET_PROFILE = '/ethernetProfile'
    SET_SETTING = '/setSetting'
    GET_SETTING = '/getSetting'
    SYSTEM_VERSION_CURRENT = '/system/version/current'
