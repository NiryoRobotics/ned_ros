import requests
import rospy


class CloudAPI(object):
    __DAILY_PATH = '/daily-reports'
    __TEST_PATH = '/test-reports'
    __ALERT_PATH = '/alert-reports'
    __AUTO_DIAGNOSIS_PATH = '/auto-diagnosis-reports'

    def __init__(self, domain, serial_number, api_key, sharing_allowed, https=False):
        self.__base_url = '{}://{}'.format('https' if https else 'http', domain)
        self.__url = self.__base_url
        self.__sharing_allowed = sharing_allowed
        self.__headers = {
            'accept': 'application/json',
            'serialNumber': serial_number,
            'apiKey': api_key
        }

    def set_serial_number(self, value):
        self.__headers['serialNumber'] = value

    def set_api_key(self, value):
        self.__headers['apiKey'] = value

    def set_sharing_allowed(self, value):
        self.__sharing_allowed = value

    @property
    def daily_reports(self):
        self.__url = self.__base_url + self.__DAILY_PATH
        return self

    @property
    def test_reports(self):
        self.__url = self.__base_url + self.__TEST_PATH
        return self

    @property
    def alert_reports(self):
        self.__url = self.__base_url + self.__ALERT_PATH
        return self

    @property
    def auto_diagnosis_reports(self):
        self.__url = self.__base_url + self.__AUTO_DIAGNOSIS_PATH
        return self

    def ping(self):
        if not self.__sharing_allowed:
            return True

        route = '/ping'
        try:
            response = requests.get(self.__url + route, headers=self.__headers)
        except requests.ConnectionError as e:
            rospy.logdebug(e)
            return False
        rospy.logdebug('Cloud API responded with code: {}'.format(response.status_code))
        return response.status_code == 200

    def send(self, payload):
        if not self.__sharing_allowed:
            return True
        try:
            response = requests.post(
                self.__url, headers=self.__headers, json=payload
            )
        except requests.ConnectionError:
            return False
        rospy.logdebug('Cloud API responded with code: {}'.format(response.status_code))
        return response.status_code == 200
