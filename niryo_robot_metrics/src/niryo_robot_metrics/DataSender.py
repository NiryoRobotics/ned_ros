import json
import urllib2
import datetime


class DataSender:
    def __init__(self, cloud_domain, serial_number, api_key):
        self.__cloud_url = 'https://{}'.format(cloud_domain)
        self.__serial_number = serial_number
        self.__api_key = api_key
        self.__timezone_offset = datetime.datetime.now(
        ).hour - datetime.datetime.utcnow().hour

    def __date_w_timezone(self, d):
        return d + '+' if self.__timezone_offset > 0 else '' + str(
            self.__timezone_offset
        ) + ':00'

    def __payload(self, data, metrics=True):
        p = {
            'type': 'health',
            'createAt':
            self.__date_w_timezone(datetime.datetime.now().isoformat()),
            'serialNumber': self.__serial_number,
            'data': {}
        }
        if metrics:
            p['data'] = [{
                'key': x.name,
                'value': x.value,
                'update_date': x.update_date
            } for x in data]
        else:
            p['data'] = [{
                'date': x.date,
                'severity': x.severity,
                'origin': x.origin,
                'message': x.message
            } for x in data]

        return p

    def __endpoint(self, route):
        print(self.__cloud_url + route)
        return self.__cloud_url + route

    def __post(self, route, data):
        try:
            request = urllib2.Request(route)
            request.add_header('Content-Type', 'application/json')
            request.add_header('apikey', self.__api_key)
            executed = urllib2.urlopen(request, json.dumps(data))
        except IOError as e:
            print(e)
            return False, False

        print(executed)
        return executed.getcode(), json.loads(executed.read())

    def send_metrics(self, metrics):
        code, response = self.__post(
            self.__endpoint('/logs'), self.__payload(metrics)
        )
        print(response)
        return code == 200

    def send_logs(self, logs):
        code, response = self.__post(
            self.__endpoint('/logs'), self.__payload(logs, metrics=False)
        )
        print(response)
        return code == 200
