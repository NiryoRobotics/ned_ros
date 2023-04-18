import json
from urllib.request import urlopen, Request
from urllib.parse import urlencode


class HttpClient:

    NONE_STATUS_CODE_MSG = 'Unable to connect to the HTTP server'

    def __init__(self, host='127.0.0.1', port=5000, prefix=None):
        self.host = host
        self.port = port
        self.prefix = prefix
        self.last_error = ''

    def __url_builder(self, uri, params=None):
        """
        Build an url from the route and parameters

        :type uri: str
        :type params: dict
        :return: None
        :rtype: str
        """
        url = 'http://{}:{}'.format(self.host, self.port)
        if self.prefix is not None:
            url += self.prefix
        url += uri

        if params:
            url += '?{}'.format(urlencode(params))

        return url

    def __payload(self, payload):
        return json.dumps(payload).encode()

    def __get(self, uri, params=None):
        url = self.__url_builder(uri, params)

        try:
            u = urlopen(url)
        except IOError:
            return False, False

        return u.getcode(), json.loads(u.read())

    def __post(self, uri, params=None):
        if params is None:
            # Important otherwise the request will be GET instead of POST
            params = {}

        url = self.__url_builder(uri)
        try:
            request = Request(url)
            request.add_header('Content-Type', 'application/json')
            executed = urlopen(request, self.__payload(params))
        except IOError:
            return False, False

        return executed.getcode(), json.loads(executed.read())

    def set_robot_name(self, name):
        status_code, response = self.__post('/setRobotName', {'name': name})

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        if status_code != 200:
            return False, response['detail']

        return True, response['detail']

    def hotspot_state(self):
        status_code, response = self.__get('/hotspotState')

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response['state']

    def wifi_state(self):
        status_code, response = self.__get('/wifiState')

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response

    def reset_wifi(self):
        status_code, response = self.__post('/resetWifi')

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response

    def reset_hotspot(self):
        status_code, response = self.__post('/resetHotspot')

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response

    def reset_ethernet(self):
        status_code, response = self.__post('/resetEthernet')

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response

    def start_hotspot(self):
        status_code, response = self.__post('/startHotspot')

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response

    def restart_wifi(self):
        status_code, response = self.__post('/restartWifi')

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response

    def start_wifi(self):
        status_code, response = self.__post('/startWifi')

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response

    def stop_wifi(self):
        status_code, response = self.__post('/stopWifi')

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response

    def stop_hotspot(self):
        status_code, response = self.__post('/stopHotspot')

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response

    def setup_ethernet(self, profile, ip="", mask="", gateway="", dns=""):
        status_code, response = self.__post('/ethernetProfile',
                                            {'profile': profile, "ip": ip, "mask": mask, "gw": gateway, "dns": dns})

        if not status_code:
            return False, self.NONE_STATUS_CODE_MSG

        return True, response
