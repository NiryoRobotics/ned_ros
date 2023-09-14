import csv
import subprocess
import rospy

from .GenericWrapper import GenericWrapper


class TuptimeWrapper(GenericWrapper):
    __available_metrics__ = [
        'system_startups',
        'average_downtime',
        'average_uptime',
        'system_life',
        'current_uptime',
        'system_shutdowns',
        'system_downtime',
        'system_uptime',
    ]

    tuptime_db_path = '/home/niryo/niryo_robot_saved_files/.config/tuptime.db'

    def _fetch_datas(self):
        # root access is sometimes needed at boot
        completed_process = subprocess.run(['sudo', 'tuptime', '-cs', '-f', self.tuptime_db_path],
                                           capture_output=True,
                                           encoding='utf-8')
        if completed_process.returncode != 0:
            raise RuntimeError('Please install the tuptime libray and give it root rights')

        reader = csv.reader(completed_process.stdout.splitlines())
        for row in reader:
            key = row.pop(0).replace(' ', '_').lower()
            if key == 'system_startups':
                self._data[key] = {
                    'name': key,
                    'value': row[0],
                    'unit': None,
                }
            elif key in ['average_downtime', 'average_uptime', 'system_life', 'current_uptime']:
                self._data[key] = {
                    'name': key,
                    'value': int(float(row[0])),
                    'unit': 'seconds',
                }
            elif key == 'system_shutdowns':
                self._data[key] = {
                    'name': key,
                    'value': int(row[0]) + int(row[3]),
                    'unit': None,
                }
            elif key in ['system_downtime', 'system_uptime']:
                self._data[key] = {
                    'name': key,
                    'value': int(float(row[2])),
                    'unit': 'seconds',
                }
