import re
import csv
import subprocess


class TuptimeWrapper:

    @property
    def data(self):
        # root access is sometimes needed at boot
        output = subprocess.check_output(['sudo', 'tuptime', '-c'])
        loutput = output.splitlines()
        reader = csv.reader(loutput)
        data = {}
        for i in reader:
            data[i[0].replace(' ', '_').lower()] = i[1::]
        return self.data_treatment(data)

    def data_treatment(self, data):
        treated_data = {}
        for key, value in data.items():
            if key == 'system_startups':
                treated_data[key] = value[0]
            elif key in ['average_downtime', 'average_uptime', 'system_life']:
                treated_data[key] = self.convert_duration(value[0])
            elif key == 'system_shutdowns':
                treated_data[key] = int(value[0]) + int(value[2])
            elif key in ['system_downtime', 'system_uptime']:
                treated_data[key] = self.convert_duration(value[1])
            elif key == 'current_uptime':
                treated_data[key] = self.convert_duration(value[0])
        return treated_data

    @staticmethod
    def convert_duration(d):
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
