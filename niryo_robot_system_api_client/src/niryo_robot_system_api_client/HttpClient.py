import json
import urllib
import urllib2


class HttpClient:

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.last_error = ''

    def __url_builder(self, uri, params=None):
        """
        Build an url from the route and parameters

        :type uri: str
        :type params: dict
        :return: None
        :rtype: str
        """
        url = 'http://{}:{}/{}'.format(self.host, self.port, uri)

        if params:
            url += '?{}'.format(urllib.urlencode(params))

        return url

    def __get(self, uri, params=None):
        url = self.__url_builder(uri, params)

        try:
            u = urllib2.urlopen(url)
        except IOError:
            return False, False

        return u.getcode(), json.loads(u.read())

    def __post(self, uri, params=None):
        if params is None:
            # Important otherwise the request will be GET instead of POST
            params = {}

        url = self.__url_builder(uri)
        try:
            request = urllib2.Request(url)
            request.add_header('Content-Type', 'application/json')
            executed = urllib2.urlopen(request, json.dumps(params))
        except IOError:
            return False, False

        return executed.getcode(), json.loads(executed.read())

    def set_robot_name(self, name):
        status_code, response = self.__post('setRobotName', {'name': name})

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        if status_code != 200:
            return False, response['detail']

        return True, response['detail']

    def hotspot_state(self):
        status_code, response = self.__get('hotspotState')

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        return True, response['state']

    def activate_hotspot(self):
        status_code, response = self.__post('switchToHotspot')

        if not status_code:
            return False, 'Unable to connect to the HTTP server'

        return True, status_code == 200
