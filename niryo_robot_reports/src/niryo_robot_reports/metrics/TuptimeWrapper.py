import re
import csv
import subprocess


class TuptimeWrapper:
    def __init__(self):
        self.__data = {}

    @property
    def data(self):
        self.__fetch_datas()
        return self.__data

    def __fetch_datas(self):
        # root access is sometimes needed at boot
        output = subprocess.check_output(['sudo', 'tuptime', '-c'])
        loutput = output.splitlines()
        reader = csv.reader(loutput)
        for row in reader:
            key = row.pop(0).replace(' ', '_').lower()
            if key == 'system_startups':
                self.__data[key] = row[0]
            elif key in ['average_downtime', 'average_uptime', 'system_life']:
                self.__data[key] = self.__convert_duration(row[0])
            elif key == 'system_shutdowns':
                self.__data[key] = int(row[0]) + int(row[2])
            elif key in ['system_downtime', 'system_uptime']:
                self.__data[key] = self.__convert_duration(row[1])
            elif key == 'current_uptime':
                self.__data[key] = self.__convert_duration(row[0])

    @staticmethod
    def __convert_duration(d):
        duration_regex = r'((?P<Y>[0-9]+)yr )?((?P<D>[0-9]+)d )?((?P<H>[0-9]+)h )?((?P<M>[0-9]+)m )?((?P<S>[0-9]+)s)'
        match = re.match(duration_regex, d)
        iso_format = 'P'
        for i in 'YD':
            if match.group(i):
                iso_format += match.group(i) + i
        iso_format += 'T'
        for i in 'HMS':
            if match.group(i):
                iso_format += match.group(i) + i

        return iso_format
