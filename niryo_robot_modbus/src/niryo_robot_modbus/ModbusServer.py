#!/usr/bin/env python
import asyncio

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusServerContext
from pymodbus.server import StartTcpServer, ServerStop
import threading
from . import logger
from .mapping import slave_context
from rospy import sleep


class ModbusServer:

    def __init__(self, address, port):
        self.address = (address, port)
        self._thread = None

        self.slave_context = slave_context
        self.context = ModbusServerContext(slaves=slave_context, single=True)

        self.identity = ModbusDeviceIdentification()
        self.identity.VendorName = 'pymodbus'
        self.identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
        self.identity.ProductName = 'pymodbus Server'
        self.identity.ModelName = 'pymodbus Server'
        self.identity.MajorMinorRevision = '1.0'

        self.slave_context.build_register()
        self.slave_context.pretty_print_registers()

    @property
    def _server_started(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self):
        if self._server_started:
            logger.info("Modbus - Server is already running.")
            return
        self._thread = threading.Thread(target=self._run_server, daemon=True)
        self._thread.start()
        logger.info("Modbus - Server Thread Started")

    def _run_server(self):
        try:
            StartTcpServer(context=self.context, identity=self.identity, address=self.address)
        except Exception as e:
            logger.error(f"Modbus - Server crashed or failed to bind to {self.address}: {e}")
        finally:
            logger.info("Modbus - Server truly closed and sockets released.")

    def stop(self):
        logger.info("Modbus - Stopping Server")
        if self._server_started:
            for _ in range(
                    3):  # Attempt to stop the server, as it may not be immediately responsive to the stop command
                try:
                    ServerStop()
                    break
                except AttributeError:
                    sleep(0.1)
                    pass

            self._thread.join()

    def change_port(self, new_port):
        self.stop()
        self.address = (self.address[0], new_port)
        logger.info(f"Modbus - Port updated to {new_port}")
        self.start()
