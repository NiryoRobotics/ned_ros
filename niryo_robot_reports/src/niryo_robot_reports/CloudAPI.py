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
    API_PREFIX = 'api/v1'
    MICROSERVICE_URI = ''

    def __init__(self, base_url, headers):
        self._base_url = f'{base_url}/{self.MICROSERVICE_URI}/{self.API_PREFIX}'
        self._headers = headers

    def update_header(self, key, value):
        self._headers[key] = value

    def ping(self):
        endpoint = f'{self._base_url}/ping'
        try:
            response = requests.get(endpoint, headers=self._headers)
        except requests.ConnectionError as e:
            raise MicroServiceError(str(e))

        if response.status_code == 404:
            raise MicroServiceError('A call to the auth service may be needed',
                                    code=MicroServiceError.Code.BAD_REQUEST_CONTENT)


class ABCReportMicroService(ABCMicroService):

    def __init__(self, base_url, header):
        super().__init__(base_url, header)
        self._base_url += f'/{self.MICROSERVICE_URI}'

    def send(self, payload):
        endpoint = self._base_url
        try:
            rospy.loginfo(f'url: {endpoint}, payload: {payload}')
            response = requests.post(endpoint, headers=self._headers, json=payload)
        except requests.ConnectionError as connection_error:
            raise MicroServiceError(str(connection_error), code=MicroServiceError.Code.CONNECTION_ERROR)
        if not (200 <= response.status_code < 400):
            raise MicroServiceError(
                (f'{self._base_url}: {self.__class__.__name__} '
                 f'responded with status {response.status_code}: {response.text}'),
                code=MicroServiceError.Code.BAD_STATUS_CODE,
            )


class AuthentificationMS(ABCMicroService):
    MICROSERVICE_URI = 'authentification'

    def authenticate(self):
        endpoint = f'{self._base_url}/token'
        try:
            response = requests.post(endpoint, headers=self._headers, json={'identifier': self._headers['identifier']})
        except requests.ConnectionError as connection_error:
            raise MicroServiceError(str(connection_error), code=MicroServiceError.Code.CONNECTION_ERROR)

        try:
            json = response.json()
        except requests.exceptions.JSONDecodeError:
            raise MicroServiceError(f'Invalid json for response {response.text}',
                                    code=MicroServiceError.Code.BAD_RESPONSE_CONTENT)
        if 400 <= response.status_code < 500:
            raise MicroServiceError(f'{endpoint}: {json["message"]}', code=MicroServiceError.Code.BAD_REQUEST_CONTENT)
        elif 500 <= response.status_code:
            raise MicroServiceError(f'{endpoint}: {json["message"]}', code=MicroServiceError.Code.UNDEFINED)

        if 'data' not in json or 'apikey' not in json['data']:
            raise MicroServiceError(f'{endpoint}: {self.__class__.__name__}: invalid data payload: {json}',
                                    code=MicroServiceError.Code.BAD_RESPONSE_CONTENT)
        return json['data']['apikey']


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

    def __init__(self, cloud_domain, serial_number, rasp_id, api_key, sharing_allowed, https=False):
        self.__base_url = '{}://{}'.format('https' if https else 'http', cloud_domain)
        self.__serial_number = serial_number
        self.__rasp_id = rasp_id
        # bypass sharing allowed
        self.__sharing_allowed = True

        self.__headers = {
            'accept': 'application/json',
            'identifier': rasp_id or serial_number,
            'apiKey': api_key,
        }
        self.__microservices = {}
        self.__init_microservices()

    def __init_microservices(self):
        self.__microservices = {
            'authentification': AuthentificationMS(self.__base_url, self.__headers),
        }
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

    def __update_identifier(self):
        identifier = self.__rasp_id or self.__serial_number
        for microservice in self.__microservices.values():
            microservice.update_header('identifier', identifier)

    def set_serial_number(self, value):
        self.__serial_number = value
        self.__update_identifier()

    def set_rasp_id(self, value):
        self.__rasp_id = value
        self.__update_identifier()

    def set_api_key(self, value):
        for microservice in self.__microservices.values():
            microservice.update_header('apiKey', value)

    def set_sharing_allowed(self, value):
        self.__sharing_allowed = value
        self.__init_microservices()

    @property
    def authentification(self):
        return self.__microservices['authentification']

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
