import csv
import subprocess

from GenericWrapper import GenericWrapper


class TuptimeWrapper(GenericWrapper):
    def _fetch_datas(self, with_unit=True):
        # root access is sometimes needed at boot
        try:
            output = subprocess.check_output(['sudo', 'tuptime', '-cs'])
        except subprocess.CalledProcessError:
            print('Please install the tuptime libray and give it root rights')
            return

        loutput = output.splitlines()
        reader = csv.reader(loutput)
        for row in reader:
            key = row.pop(0).replace(' ', '_').lower()
            if key == 'system_startups':
                self._data.append({
                    'name': key,
                    'value': row[0],
                    'unit': None,
                })
            elif key in ['average_downtime', 'average_uptime', 'system_life', 'current_uptime']:
                self._data.append({
                    'name': key,
                    'value': int(float(row[0])),
                    'unit': 'seconds',
                })
            elif key == 'system_shutdowns':
                self._data.append({
                    'name': key,
                    'value': int(row[0]) + int(row[3]),
                    'unit': None,
                })
            elif key in ['system_downtime', 'system_uptime']:
                self._data.append({
                    'name': key,
                    'value': int(float(row[2])),
                    'unit': 'seconds',
                })
