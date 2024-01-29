#!/usr/bin/env python
import asyncio

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusServerContext
from pymodbus.server import StartAsyncTcpServer, ServerStop

from . import logger
from .mapping import slave_context


class ModbusServer:

    def __init__(self, address, port):
        self.address = (address, port)

        self.slave_context = slave_context
        self.context = ModbusServerContext(slaves=slave_context, single=True)

        self.identity = ModbusDeviceIdentification()
        self.identity.VendorName = 'pymodbus'
        self.identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
        self.identity.ProductName = 'pymodbus Server'
        self.identity.ModelName = 'pymodbus Server'
        self.identity.MajorMinorRevision = '1.0'

        self.__server_started = False

    def start(self):
        self.slave_context.build_register()
        self.slave_context.pretty_print_registers()
        asyncio.run(self.__start_server())
        logger.info("Modbus - Server Started")

    async def __start_server(self):
        self.__server_started = True
        await StartAsyncTcpServer(context=self.context, identity=self.identity, address=self.address)

    def stop(self):
        logger.info("Modbus - Closing Server")
        if self.__server_started:
            ServerStop()
