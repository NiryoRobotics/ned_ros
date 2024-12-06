import rospy
from threading import Thread


class PromiseServiceProxy:

    def __init__(self, service_name, service_type, callback=None):
        self.__service_proxy = rospy.ServiceProxy(service_name, service_type)
        self.__callback = callback
        self.__wait_thread = Thread(target=self.__wait_for_service)
        self.__wait_thread.start()

    def __wait_for_service(self):
        self.__service_proxy.wait_for_service()
        if self.__callback is None:
            return
        try:
            self.__callback(self.__service_proxy)
        except Exception as e:
            rospy.logerr(f'Error in {self.__service_proxy.resolved_name} promise callback: {e}')

    def __check_availability(self):
        try:
            self.__service_proxy.wait_for_service(timeout=5)
        except rospy.ROSException:
            raise RuntimeError(f'{self.__service_proxy.resolved_name} is not available.') from None

    @property
    def available(self):
        return not self.__wait_thread.is_alive()

    def __call__(self, *args, **kwargs):
        self.__check_availability()
        return self.__service_proxy(*args, **kwargs)

    def __getattr__(self, item):
        self.__check_availability()
        return getattr(self.__service_proxy, item)
