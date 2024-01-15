from niryo_robot_modbus.ModbusServer import ModbusServer
if __name__ == "__main__":
    server = ModbusServer('0.0.0.0', 5020)
    try:
        server.start()
    except KeyboardInterrupt:
        server.stop()