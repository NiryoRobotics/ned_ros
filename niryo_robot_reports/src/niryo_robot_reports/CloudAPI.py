import json

import requests
import rospy


class CloudAPI:

    __DAILY_PATH = '/daily-reports'
    __TEST_PATH = '/test-reports'

    def __init__(self, domain, serial_number, api_key, https=False):
        self.__base_url = '{}://{}'.format('https' if https else 'http', domain)
        self.__url = self.__base_url
        self.__headers = {
            'accept': 'application/json',
            'serialNumber': serial_number,
            'apiKey': api_key
        }

    @property
    def daily_reports(self):
        self.__url = self.__base_url + self.__DAILY_PATH
        return self

    @property
    def test_reports(self):
        self.__url = self.__base_url + self.__TEST_PATH
        return self

    def ping(self):
        route = '/ping'
        try:
            response = requests.get(self.__url + route, headers=self.__headers)
        except requests.ConnectionError:
            return False
        rospy.logdebug('Cloud API responded with code: {}'.format(response.status_code))
        return response.status_code == 200

    def send(self, payload):
        try:
            response = requests.post(
                self.__url, headers=self.__headers, json=payload
            )
        except requests.ConnectionError:
            return False
        rospy.logdebug('Cloud API responded with code: {}'.format(response.status_code))
        return response.status_code == 200
