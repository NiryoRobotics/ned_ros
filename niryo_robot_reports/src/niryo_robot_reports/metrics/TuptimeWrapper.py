import csv
import subprocess

from GenericWrapper import GenericWrapper


class TuptimeWrapper(GenericWrapper):
    def _fetch_datas(self):
        # root access is sometimes needed at boot
        output = subprocess.check_output(['sudo', 'tuptime', '-cs'])
        loutput = output.splitlines()
        reader = csv.reader(loutput)
        for row in reader:
            key = row.pop(0).replace(' ', '_').lower()
            if key == 'system_startups':
                self._data[key] = row[0]
            elif key in ['average_downtime', 'average_uptime', 'system_life']:
                self._data[key] = int(row[0])
            elif key == 'system_shutdowns':
                self._data[key] = int(row[0]) + int(row[3])
            elif key in ['system_downtime', 'system_uptime']:
                self._data[key] = int(row[1])
            elif key == 'current_uptime':
                self._data[key] = int(row[0])
