import requests
import rospy


class HttpClient:

    ERROR_MSG = 'Error while requesting the HTTP server'

    def __init__(self, host='127.0.0.1', port=5000, prefix='/api/v2'):
        self.__base_url = f'http://{host}:{port}'
        if prefix is not None:
            self.__base_url += f'{prefix}'

    def __parse_response(self, response=None):
        default_response = False, '', {}
        if response is None or response.status_code != 200:
            return default_response
        try:
            json = response.json()
        except requests.exceptions.JSONDecodeError as json_decode_error:
            rospy.logerr(json_decode_error)
            return default_response
        return True, json['detail'], json['data']

    def __get(self, uri):
        endpoint = f'{self.__base_url}{uri}'

        try:
            response = requests.get(endpoint)
        except requests.ConnectionError as connection_error:
            rospy.logerr_throttle_identical(60, str(connection_error))
            response = None

        return self.__parse_response(response)

    def __post(self, uri, params=None):
        if params is None:
            params = {}

        endpoint = f'{self.__base_url}{uri}'

        try:
            response = requests.post(endpoint, json=params)
        except requests.ConnectionError as connection_error:
            rospy.logerr_throttle_identical(60, str(connection_error))
            response = None

        return self.__parse_response(response)

    def set_robot_name(self, name):
        success, detail, _ = self.__post('/setRobotName', {'name': name})

        if not success:
            return False, self.ERROR_MSG

        return True, detail

    def hotspot_state(self):
        success, _, data = self.__get('/hotspotState')

        if not success:
            return False, self.ERROR_MSG

        return True, data

    def wifi_state(self):
        status_code, _, data = self.__get('/wifiState')

        if not status_code:
            return False, self.ERROR_MSG

        return True, data

    def reset_wifi(self):
        status_code, detail, _ = self.__post('/resetWifi')

        if not status_code:
            return False, self.ERROR_MSG

        return True, detail

    def reset_hotspot(self):
        status_code, detail, _ = self.__post('/resetHotspot')

        if not status_code:
            return False, self.ERROR_MSG

        return True, detail

    def reset_ethernet(self):
        status_code, detail, _ = self.__post('/resetEthernet')

        if not status_code:
            return False, self.ERROR_MSG

        return True, detail

    def start_hotspot(self):
        status_code, detail, _ = self.__post('/startHotspot')

        if not status_code:
            return False, self.ERROR_MSG

        return True, detail

    def restart_wifi(self):
        status_code, detail, _ = self.__post('/restartWifi')

        if not status_code:
            return False, self.ERROR_MSG

        return True, detail

    def start_wifi(self):
        status_code, detail, _ = self.__post('/startWifi')

        if not status_code:
            return False, self.ERROR_MSG

        return True, detail

    def stop_wifi(self):
        status_code, detail, _ = self.__post('/stopWifi')

        if not status_code:
            return False, self.ERROR_MSG

        return True, detail

    def stop_hotspot(self):
        status_code, detail, _ = self.__post('/stopHotspot')

        if not status_code:
            return False, self.ERROR_MSG

        return True, detail

    def setup_ethernet(self, profile, ip="", mask="", gateway="", dns=""):
        (status_code, detail, _) = self.__post(
            '/ethernetProfile',
            {
                'profile': profile,
                "ip": ip,
                "mask": mask,
                "gw": gateway,
                "dns": dns,
            },
        )

        if not status_code:
            return False, self.ERROR_MSG

        return True, detail
