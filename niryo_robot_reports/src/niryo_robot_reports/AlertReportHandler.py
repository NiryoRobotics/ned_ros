# coding=utf-8
import time

from enum import Enum
from threading import Thread
from datetime import datetime

import rospy
from niryo_robot_metrics.PsutilWrapper import PsutilWrapper
from niryo_robot_reports.CloudAPI import MicroServiceError


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
        return '{} {} with value {}'.format(self.metric_name, 'succeeded' if self.__success else 'failed', self.__value)

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
        self.__watch = Thread(target=self.__run)
        self.__watch.setDaemon(True)

        self.__psutil_wrapper = PsutilWrapper()

        self.__metrics = {
            CheckFrequencies.LOW: [],
            CheckFrequencies.NORMAL: [],
            CheckFrequencies.HIGH: [],
        }
        for alert_report_name, alert_report_args in rospy.get_param('~alert_report').items():
            freq = CheckFrequencies[alert_report_args['frequency']]
            threshold = alert_report_args['threshold']
            self.__metrics[freq].append(
                MetricChecker(metrics=getattr(self.__psutil_wrapper, alert_report_name), test=lambda x: x < threshold))

        self.__check_by_frequency(CheckFrequencies.LOW)
        self.__watch.start()

    def __check_by_frequency(self, frequency):
        for metric_checker in self.__metrics[frequency]:
            success = metric_checker.test()
            if success:
                continue
            self.__send_report(metric_checker)

    def __send_report(self, metric_checker):
        try:
            self.__cloud_api.alert_reports.send({
                'metric': metric_checker.metric_name, 'value': metric_checker.value, 'date': datetime.now().isoformat()
            })
        except MicroServiceError as microservice_error:
            rospy.logerr(str(microservice_error))

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
                        if not test.test():
                            self.__send_report(test)
            time.sleep(clock_frequency)
