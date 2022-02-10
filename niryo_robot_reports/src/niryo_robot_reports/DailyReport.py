from Report import Report


class DailyReport(Report):
    DEFAULT_VALUE = {'logs': []}

    def add_log(self, msg, log_type, date):
        inserted = False
        for index, log in enumerate(self.content['logs']):
            if log['msg'] == msg:
                self.content['logs'][index]['occurred'].append(date)
                inserted = True
                break
        if not inserted:
            self.content['logs'].append({
                'msg': msg,
                'type': log_type,
                'occurred': [date]
            })
        self._write()

    def set_date(self, date):
        self.content['date'] = date
        self._write()
