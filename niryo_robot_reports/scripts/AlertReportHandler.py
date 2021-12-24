# coding=utf-8
import time

from enum import Enum
from threading import Event
from datetime import datetime

from niryo_robot_reports.metrics.PsutilWrapper import PsutilWrapper


class CheckFrequencies(Enum):
    VERY_LOW = 0
    LOW = 600
    NORMAL = 5
    HIGH = 1

    FREQUENCIES = {
        LOW: None,
        NORMAL: 5,
        HIGH: 1,
    }


class MetricChecker:
    def __init__(self, metrics, test):
        self.__metrics = metrics
        self.__test = test
        self.__value = None

    @property
    def metric_name(self):
        # Return the name of the function without the 'get_' prefix
        return self.__metrics.__name__[3::]

    @property
    def value(self):
        return self.__value

    def test(self):
        self.__value = self.__metrics()
        return self.__test(self.__value)


class AlertReportHandler:
    def __init__(self, cloud_api):
        self.__cloud_api = cloud_api
        self.__stop_watch = Event()

        self.__psutil_wrapper = PsutilWrapper()
        self.__metrics = {
            CheckFrequencies.VERY_LOW: [
                MetricChecker(self.__psutil_wrapper.get_rom_usage, lambda x: x < 90)
            ],
            CheckFrequencies.HIGH: [
                MetricChecker(self.__psutil_wrapper.get_cpu_usage, lambda x: x < 80),
                MetricChecker(self.__psutil_wrapper.get_cpu_temperature, lambda x: x < 50),
                MetricChecker(self.__psutil_wrapper.get_ram_usage, lambda x: x < 75),
            ],
        }

        self.check_by_frequency(CheckFrequencies.VERY_LOW)

    def check_by_frequency(self, frequency):
        for metric_checker in self.__metrics[frequency]:
            success = metric_checker.test()
            if success:
                continue
            self.__cloud_api.alert_report.send({
                'metric': metric_checker.metric_name,
                'value': metric_checker.value,
                'date': datetime.now()
            })

    def quit(self):
        self.__stop_watch.set()

    def run(self):
        clock_frequency = 1
        frequencies_counter = {
            CheckFrequencies.LOW: 0,
            CheckFrequencies.NORMAL: 0,
            CheckFrequencies.HIGH: 0,
        }
        while not self.__stop_watch.wait(clock_frequency):
            frequencies_counter = {x: y + 1 for x, y in frequencies_counter.items()}
            for frequency, counter in frequencies_counter.items():
                if counter == frequency:
                    for test in self.__metrics[frequency]:
                        print(test.metric_name)
            time.sleep(clock_frequency)
