import requests


class SystemApiResponse:

    def __init__(self, status_code, code, detail, data):
        self._status_code = status_code
        self._code = code
        self._detail = detail
        self._data = data

    @classmethod
    def from_requests(cls, requests_response):
        try:
            response_body = requests_response.json()
        except requests.exceptions.JSONDecodeError:
            return cls(status_code=requests_response.status_code)

        try:
            return cls(requests_response.status_code,
                       response_body['code'],
                       response_body['detail'],
                       response_body['data'])
        except KeyError as key_error:
            raise ValueError(f'Missing key in response payload: "{str(key_error)}"')

    @property
    def status_code(self):
        return self._status_code

    @property
    def code(self):
        return self._code

    @property
    def detail(self):
        return self._detail

    @property
    def data(self):
        return self._data

    @property
    def success(self):
        return 200 <= self._status_code < 400
