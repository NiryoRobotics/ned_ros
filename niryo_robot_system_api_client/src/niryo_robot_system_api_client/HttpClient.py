import json
import urllib
import sys

if sys.version_info[0] == 3:
    from urllib.request import urlopen
else:
    # Not Python 3 - today, it is most likely to be Python 2
    # But note that this might need an update when Python 4
    # might be around one day
    from urllib import urlopen


class HttpClient:

    def __init__(self, host, port, prefix=None):
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
            url += '?{}'.format(urllib.urlencode(params))

        return url

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

        url = self.__url_builder(uri).encode()
        try:
            request = urllib.request.Request(url)
            request.add_header('Content-Type', 'application/json')
            executed = urlopen(request, json.dumps(params))
        except IOError:
            return False, False

        return executed.getcode(), json.loads(executed.read())

    def set_robot_name(self, name):
        status_code, response = self.__post('/setRobotName', {'name': name})

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        if status_code != 200:
            return False, response['detail']

        return True, response['detail']

    def hotspot_state(self):
        status_code, response = self.__get('/hotspotState')

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        return True, response['state']

    def wifi_state(self):
        status_code, response = self.__get('/wifiState')

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        return True, response

    def activate_hotspot(self):
        status_code, response = self.__post('/switchToHotspot')

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        return True, response

    def restart_wifi(self):
        status_code, response = self.__post('/restartWifi')

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        return True, response

    def deactivate_wifi(self):
        status_code, response = self.__post('/deactivateWifi')

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        return True, response

    def reconnect_last_wifi(self):
        status_code, response = self.__post('/reconnectLastWifi')

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        return True, response

    def setup_ethernet(self, profile, ip="", mask="", gateway="", dns=""):
        status_code, response = self.__post('/ethernetProfile',
                                            {'profile': profile, "ip": ip, "mask": mask, "gw": gateway, "dns": dns})

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        return True, response
