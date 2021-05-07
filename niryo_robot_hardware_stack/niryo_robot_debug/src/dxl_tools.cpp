/*
    dxl_tools.cpp
    Copyright (C) 2018 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "niryo_robot_debug/dxl_tools.h"

namespace robotDebug {

    /**
     * @brief DxlTools::DxlTools
     */
    DxlTools::DxlTools()
    {}

    /**
     * @brief DxlTools::DxlTools
     * @param portHandler
     * @param packetHandler
     */
    DxlTools::DxlTools(std::shared_ptr<dynamixel::PortHandler> portHandler,
                       std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
        _portHandler(portHandler),
        _packetHandler(packetHandler)
    {
    }

    /**
     * @brief DxlTools::setupDxlBus
     * @param baudrate
     * @return
     */
    int DxlTools::setupDxlBus(int baudrate)
    {
        if (!_portHandler->setupGpio()) {
            printf("ERROR: Failed to setup direction GPIO pin for Dynamixel half-duplex serial\n");
            return -1;
        }
        if (!_portHandler->openPort()) {
            printf("Error: Failed to open Uart port for Dynamixel bus\n");
            return -1;
        }
        if (!_portHandler->setBaudRate(baudrate)) {
            printf("Error: Failed to set baudrate for Dynamixel bus\n");
            return -1;
        }

        printf("Dxl Bus successfully setup\n");
        return 1;
    }

    /**
     * @brief DxlTools::broadcastPing
     */
    void DxlTools::broadcastPing()
    {
        int dxl_comm_result = COMM_TX_FAIL;
        std::vector<uint8_t> id_list;

        dxl_comm_result = _packetHandler->broadcastPing(_portHandler.get(), id_list);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("Failed to scan Dynamixel bus: %d\n", dxl_comm_result);
            return;
        }

        printf("Detected Dxl motor IDs:\n");
        for (uint8_t id : id_list) {
            printf("- %d\n", id);
        }
    }

    /**
     * @brief DxlTools::ping
     * @param id
     */
    void DxlTools::ping(int id)
    {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;
        uint16_t dxl_model_number;

        dxl_comm_result = _packetHandler->ping(_portHandler.get(), (uint8_t)id, &dxl_model_number, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("Ping failed: %d\n", dxl_comm_result);
        }
        else if (dxl_error != 0) {
            printf("Ping OK for ID: %d, but an error flag is set on the motor: %d\n", id, static_cast<int>(dxl_error));
        }
        else {
            printf("Ping succeeded for ID: %d\n", id);
        }
    }

    /**
     * @brief DxlTools::setRegister
     * @param id
     * @param reg_address
     * @param value
     * @param byte_number
     */
    int DxlTools::setRegister(uint8_t id, uint8_t reg_address,
                               uint32_t value, uint8_t byte_number)
    {
        int dxl_comm_result = COMM_TX_FAIL;

        switch(byte_number) {
            case 1:
                dxl_comm_result = _packetHandler->write1ByteTxOnly(_portHandler.get(), id,
                        reg_address, (uint8_t)value);
            break;
            case 2:
                dxl_comm_result = _packetHandler->write2ByteTxOnly(_portHandler.get(), id,
                        reg_address, (uint16_t)value);
            break;
            case 4:
                dxl_comm_result = _packetHandler->write4ByteTxOnly(_portHandler.get(), id,
                        reg_address, (uint32_t)value);
            break;
            default:
                printf("ERROR: Size param must be 1, 2 or 4 bytes\n");
            break;
        }

        return dxl_comm_result;

    }

    /**
     * @brief DxlTools::getRegister
     * @param id
     * @param reg_address
     * @param value
     * @param byte_number
     * @return
     */
    int DxlTools::getRegister(uint8_t id, uint8_t reg_address, uint32_t &value, uint8_t byte_number)
    {
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;

        switch(byte_number) {
            case 1:
            {
                uint8_t read_data;
                dxl_comm_result = _packetHandler->read1ByteTxRx(_portHandler.get(), id,
                                            reg_address, &read_data, &dxl_error);
                value = read_data;
            }
            break;
            case 2:
            {
                uint16_t read_data;
                dxl_comm_result = _packetHandler->read2ByteTxRx(_portHandler.get(), id,
                                            reg_address, &read_data, &dxl_error);
                value = read_data;
            }
            break;
            case 4:
            {
                uint32_t read_data;
                dxl_comm_result = _packetHandler->read4ByteTxRx(_portHandler.get(), id,
                                            reg_address, &read_data, &dxl_error);
                value = read_data;
            }
            break;
            default:
                printf("ERROR: Size param must be 1, 2 or 4 bytes\n");
            break;
        }

        if (0 != dxl_error)
            dxl_comm_result = dxl_error;

        return dxl_comm_result;
    }

    /**
     * @brief DxlTools::closePort
     */
    void DxlTools::closePort()
    {
        _portHandler->closePort();
    }
}
