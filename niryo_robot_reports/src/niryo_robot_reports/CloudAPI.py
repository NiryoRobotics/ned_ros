import requests
import rospy
from abc import ABC
from enum import Enum


class MicroServiceError(Exception):

    class Code(Enum):
        UNDEFINED = 0
        CONNECTION_ERROR = 1
        BAD_STATUS_CODE = 2
        BAD_REQUEST_CONTENT = 3
        BAD_RESPONSE_CONTENT = 4

    def __init__(self, message, code=Code.UNDEFINED):
        self.code = code
        super().__init__(message)


class ABCMicroService(ABC):
    MICROSERVICE_URI = ''
    API_PREFIX = 'api/v1'

    def __init__(self, base_url, headers):
        self._base_url = f'{base_url}/{self.MICROSERVICE_URI}/{self.API_PREFIX}'
        self._headers = headers

    def update_header(self, key, value):
        self._headers[key] = value


class ABCReportMicroService(ABCMicroService):

    def ping(self):
        endpoint = f'{self._base_url}/{self.MICROSERVICE_URI}/ping'
        try:
            response = requests.get(endpoint, headers=self._headers)
        except requests.ConnectionError as e:
            rospy.logdebug(e)
            return False
        rospy.logdebug('Cloud API responded with code: {}'.format(response.status_code))
        return response.status_code == 200

    def send(self, payload):
        try:
            response = requests.post(self._base_url, headers=self._headers, json=payload)
        except requests.ConnectionError as connection_error:
            raise MicroServiceError(str(connection_error), code=MicroServiceError.Code.CONNECTION_ERROR)
        rospy.logdebug('Cloud API responded with code: {}'.format(response.status_code))
        if response.status_code != 200:
            raise MicroServiceError(f'MicroService responded with status {response.status_code}: {response.text}',
                                    code=MicroServiceError.Code.BAD_STATUS_CODE)


class AuthentificationMS(ABCMicroService):
    MICROSERVICE_URI = 'authentification'

    def call(self):
        url = f'{self._base_url}/{self._headers["raspId"]}'
        try:
            response = requests.post(url, headers=self._headers)
        except requests.ConnectionError as connection_error:
            raise MicroServiceError(str(connection_error), code=MicroServiceError.Code.CONNECTION_ERROR)
        rospy.logdebug('Cloud API responded with code: {}'.format(response.status_code))
        if response.status_code != 200:
            if response.status_code == 400:
                raise MicroServiceError(f'There is no robot registered with the rasp id "{self._headers["raspId"]}',
                                        code=MicroServiceError.Code.BAD_REQUEST_CONTENT)
            raise MicroServiceError(f'MicroService responded with status {response.status_code}: {response.text}',
                                    code=MicroServiceError.Code.BAD_STATUS_CODE)
        json_data = response.json()['data']
        if 'apiKey' not in json_data:
            raise MicroServiceError(f'apiKey not in the response', code=MicroServiceError.Code.BAD_RESPONSE_CONTENT)
        return json_data['apiKey']


class DailyReportMS(ABCReportMicroService):
    MICROSERVICE_URI = 'daily-reports'


class TestReportMS(ABCReportMicroService):
    MICROSERVICE_URI = 'test-reports'


class AlertReportMS(ABCReportMicroService):
    MICROSERVICE_URI = 'alert-reports'


class AutoDiagnosisReportMS(ABCReportMicroService):
    MICROSERVICE_URI = 'auto-diagnosis-reports'


class FakeReportMS(ABCReportMicroService):

    def send(self, payload):
        return True


class CloudAPI(object):

    def __init__(self, domain, serial_number, rasp_id, api_key, sharing_allowed, https=False):
        self.__base_url = '{}://{}'.format('https' if https else 'http', domain)
        self.__url = self.__base_url
        self.__sharing_allowed = sharing_allowed
        self.__headers = {
            'accept': 'application/json',
            'serialNumber': serial_number,
            'raspId': rasp_id,
            'apiKey': api_key,
        }
        self.__microservices = {}
        self.__init_microservices()

        if not api_key and rasp_id:
            try:
                api_key = self.__microservices['authentification'].call()
                self.set_api_key(api_key)
            except MicroServiceError as microservice_error:
                rospy.logerr(microservice_error)

    def __init_microservices(self):
        self.__microservices = {'authentification': AuthentificationMS(self.__base_url, self.__headers)}
        if self.__sharing_allowed:
            self.__microservices.update({
                'daily_reports': DailyReportMS(self.__base_url, self.__headers),
                'test_reports': TestReportMS(self.__base_url, self.__headers),
                'alert_reports': AlertReportMS(self.__base_url, self.__headers),
                'auto_diagnosis_reports': AutoDiagnosisReportMS(self.__base_url, self.__headers)
            })
        else:
            self.__microservices.update({
                'daily_reports': FakeReportMS(self.__base_url, self.__headers),
                'test_reports': FakeReportMS(self.__base_url, self.__headers),
                'alert_reports': FakeReportMS(self.__base_url, self.__headers),
                'auto_diagnosis_reports': FakeReportMS(self.__base_url, self.__headers)
            })

    def set_serial_number(self, value):
        for microservice in self.__microservices.values():
            microservice.update_header('serialNumber', value)

    def set_rasp_id(self, value):
        for microservice in self.__microservices.values():
            microservice.update_header('raspId', value)

    def set_api_key(self, value):
        for microservice in self.__microservices.values():
            microservice.update_header('apiKey', value)

    def set_sharing_allowed(self, value):
        self.__sharing_allowed = value
        self.__init_microservices()

    @property
    def daily_reports(self):
        return self.__microservices['daily_reports']

    @property
    def test_reports(self):
        return self.__microservices['test_reports']

    @property
    def alert_reports(self):
        return self.__microservices['alert_reports']

    @property
    def auto_diagnosis_reports(self):
        return self.__microservices['auto_diagnosis_reports']
