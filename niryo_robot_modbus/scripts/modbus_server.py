#!/usr/bin/env python

from select import error
import rospy
import socket
from threading import Thread

from pymodbus.server.sync import ModbusTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext

from coil_data_block import CoilDataBlock, CoilDataBlockNed2
from discrete_input_data_block import DiscreteInputDataBlock, DiscreteInputDataBlockNed2
from input_register_data_block import InputRegisterDataBlock
from holding_register_data_block import HoldingRegisterDataBlock


class ModbusServer:

    def __init__(self, address, port):

        if rospy.get_param("~hardware_version") == "ned2":
            self.coil = CoilDataBlockNed2()
            self.discrete_input = DiscreteInputDataBlockNed2()
        else:
            self.coil = CoilDataBlock()
            self.discrete_input = DiscreteInputDataBlock()

        self.input_register = InputRegisterDataBlock()
        self.holding_register = HoldingRegisterDataBlock()
        self.store = ModbusSlaveContext(di=self.discrete_input,
                                        co=self.coil, hr=self.holding_register, ir=self.input_register)
        self.context = ModbusServerContext(slaves=self.store, single=True)

        self.identity = ModbusDeviceIdentification()
        self.identity.VendorName = 'pymodbus'
        self.identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
        self.identity.ProductName = 'pymodbus Server'
        self.identity.ModelName = 'pymodbus Server'
        self.identity.MajorMinorRevision = '1.0'

        try:
            self.server = ModbusTcpServer(context=self.context,
                                          framer=None, identity=self.identity, address=(address, port))

        except socket.error as err:
            rospy.logerr("ModbusServer.init : TCP server unable to start : %s", err)
            self.server = None

    def start(self):
        t = Thread(target=self.__start_server)
        t.start()

    def __start_server(self):
        self.discrete_input.start_ros_subscribers()
        self.input_register.start_ros_subscribers()
        if self.server is not None:
            self.server.serve_forever()

    def stop(self):
        rospy.loginfo("Modbus - Stopping ROS subscribers")
        self.discrete_input.stop_ros_subscribers()
        self.input_register.stop_ros_subscribers()

        if self.server is not None:
            rospy.loginfo("Modbus - Closing Server")
            self.server.server_close()
            self.server.shutdown()
