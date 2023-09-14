from niryo_robot_reports.Report import Report


class DailyReport(Report):
    DEFAULT_VALUE = {'logs': [], 'metrics': {}}

    def add_log(self, msg, log_type, date):
        inserted = False
        for index, log in enumerate(self.content['logs']):
            if log['msg'] == msg:
                self.content['logs'][index]['occurred'].append(date)
                inserted = True
                break
        if not inserted:
            self.content['logs'].append({'msg': msg, 'type': log_type, 'occurred': [date]})
        self._write()

    def add_metrics(self, metric_name, metric_value):
        if metric_name in self.content['metrics'] and isinstance(self.content['metrics'][metric_name], list):
            self.content['metrics'][metric_name].append(metric_value)
        self.content['metrics'][metric_name] = metric_value
        self._write()

    def set_date(self, date):
        self.content['date'] = date
        self._write()
