import json

import requests


class CloudAPI:

    __DAILY_PATH = '/daily-reports'
    __TEST_PATH = '/test-reports'
    __ALERT_PATH = '/alert-report'

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
    def test_report(self):
        self.__url = self.__base_url + self.__TEST_PATH
        return self

    @property
    def alert_report(self):
        self.__url = self.__base_url + self.__ALERT_PATH
        return self

    def ping(self):
        route = '/ping'
        response = requests.get(self.__url + route, headers=self.__headers)
        return response.status_code == 200

    def send(self, payload):
        response = requests.post(
            self.__url, headers=self.__headers, json=payload
        )
        return response.status_code == 200
