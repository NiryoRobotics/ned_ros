from niryo_robot_reports.Report import Report


class TestReport(Report):
    def set_content(self, content):
        self.content = content
        self._write()
