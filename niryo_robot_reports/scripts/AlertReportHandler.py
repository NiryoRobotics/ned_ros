# coding=utf-8
import time

from enum import Enum
from threading import Event, Thread
from datetime import datetime

from niryo_robot_reports.metrics.PsutilWrapper import PsutilWrapper


class CheckFrequencies(Enum):
    LOW = 0
    NORMAL = 60
    HIGH = 1


class MetricChecker:
    def __init__(self, metrics, test):
        self.__metrics = metrics
        self.__test = test
        self.__value = None
        self.__success = None

    def __str__(self):
        return '{} {} with value {}'.format(
            self.metric_name,
            'succeed' if self.__success else 'failed',
            self.__value
        )

    @property
    def metric_name(self):
        # Return the name of the function without the 'get_' prefix
        return self.__metrics.__name__[4::]

    @property
    def value(self):
        return self.__value

    @property
    def success(self):
        return self.__success

    def test(self):
        self.__value = self.__metrics()
        self.__success = self.__test(self.__value)
        return self.__success


class AlertReportHandler:
    def __init__(self, cloud_api):
        self.__cloud_api = cloud_api
        self.__stop_watch = Event()
        self.__watch = Thread(target=self.__run)
        self.__watch.setDaemon(True)

        self.__psutil_wrapper = PsutilWrapper()
        self.__metrics = {
            CheckFrequencies.LOW: [
                MetricChecker(self.__psutil_wrapper.get_rom_usage, lambda x: x < 90)
            ],
            CheckFrequencies.HIGH: [
                MetricChecker(self.__psutil_wrapper.get_cpu_usage, lambda x: x < 80),
                MetricChecker(self.__psutil_wrapper.get_cpu_temperature, lambda x: x < 50),
                MetricChecker(self.__psutil_wrapper.get_ram_usage, lambda x: x < 75),
            ],
        }

        self.__check_by_frequency(CheckFrequencies.LOW)
        self.__watch.start()

    def __check_by_frequency(self, frequency):
        for metric_checker in self.__metrics[frequency]:
            success = metric_checker.test()
            if success:
                continue
            self.__send_report(metric_checker)

    def __send_report(self, metric_checker):
        print(metric_checker)
        self.__cloud_api.alert_report.send({
            'metric': metric_checker.metric_name,
            'value': metric_checker.value,
            'date': datetime.now()
        })

    def __run(self):
        clock_frequency = 1
        frequencies_counter = {
            CheckFrequencies.LOW: 0,
            CheckFrequencies.NORMAL: 0,
            CheckFrequencies.HIGH: 0,
        }
        while True:
            # All this process takes less than ~10ms, which is negligible
            frequencies_counter = {x: y + 1 for x, y in frequencies_counter.items()}
            for frequency, counter in frequencies_counter.items():
                if counter == frequency.value:
                    frequencies_counter[frequency] = 0
                    for test in self.__metrics[frequency]:
                        print(test)
                        if not test.test():
                            self.__send_report(test)
            time.sleep(clock_frequency)
