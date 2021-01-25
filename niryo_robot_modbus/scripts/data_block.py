#!/usr/bin/env python

import rospy

from pymodbus.datastore import ModbusSparseDataBlock


class DataBlock(ModbusSparseDataBlock):

    def __init__(self):
        super(DataBlock, self).__init__(values=[0] * 1000)

    # Called from internal functions
    # Modbus addresses start at 1
    # There is an offset with what the client is asking
    def setValuesOffset(self, address, values):
        self.setValues(address + 1, values)

    def getValuesOffset(self, address, count=1):
        return self.getValues(address + 1, count)

    @staticmethod
    def call_ros_service(service_name, service_msg_type, *args):
        # Connect to service
        try:
            rospy.wait_for_service(service_name, 0.1)
        except rospy.ROSException, e:
            return

            # Call service
        try:
            service = rospy.ServiceProxy(service_name, service_msg_type)
            response = service(*args)
            return response
        except rospy.ServiceException, e:
            return
