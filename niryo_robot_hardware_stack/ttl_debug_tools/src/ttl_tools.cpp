/*
    ttl_tools.cpp
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
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#include <memory>
#include <utility>
#include <vector>

#include "dynamixel_sdk/packet_handler.h"
#include "ttl_debug_tools/ttl_tools.h"

namespace ttl_debug_tools
{

/**
 * @brief TtlTools::TtlTools
 */
TtlTools::TtlTools() = default;

/**
 * @brief TtlTools::TtlTools
 * @param portHandler
 * @param packetHandler
 */
TtlTools::TtlTools(std::shared_ptr<dynamixel::PortHandler> portHandler, std::shared_ptr<dynamixel::PacketHandler> packetHandler)
    : _portHandler(std::move(portHandler)), _packetHandler(std::move(packetHandler))
{
}

/**
 * @brief TtlTools::setupTtlBus
 * @param baudrate
 * @return
 */
int TtlTools::setupBus(int baudrate)
{
    if (!_portHandler->openPort())
    {
        printf("Error: Failed to open Uart port for Dynamixel bus\n");
        return -1;
    }
    if (!_portHandler->setBaudRate(baudrate))
    {
        printf("Error: Failed to set baudrate for Dynamixel bus\n");
        return -1;
    }

    printf("Ttl Bus successfully setup\n");
    return 1;
}

/**
 * @brief TtlTools::broadcastPing
 */
void TtlTools::broadcastPing()
{
    int comm_result = COMM_TX_FAIL;
    std::vector<uint8_t> id_list;

    comm_result = _packetHandler->broadcastPing(_portHandler.get(), id_list);
    if (comm_result != COMM_SUCCESS)
    {
        printf("Failed to scan Dynamixel bus: %d\n", comm_result);
        return;
    }

    printf("Detected Ttl motor IDs:\n");
    for (uint8_t id : id_list)
    {
        printf("- %d\n", id);
    }
}

/**
 * @brief TtlTools::ping
 * @param id
 */
void TtlTools::ping(int id)
{
    int comm_result = COMM_TX_FAIL;
    uint8_t error = 0;
    uint16_t model_number;

    comm_result = _packetHandler->ping(_portHandler.get(), static_cast<uint8_t>(id), &model_number, &error);
    if (comm_result != COMM_SUCCESS)
    {
        printf("Ping failed: %d\n", comm_result);
    }
    else if (error != 0)
    {
        printf("Ping OK for ID: %d, but an error flag is set on the motor: %d\n", id, static_cast<int>(error));
    }
    else
    {
        printf("Ping succeeded for ID: %d\n", id);
    }
}

/**
 * @brief TtlTools::setRegister
 * @param id
 * @param reg_address
 * @param value
 * @param byte_number
 */
int TtlTools::setRegister(uint8_t id, uint16_t reg_address, uint32_t value, uint8_t byte_number)
{
    int comm_result = COMM_TX_FAIL;

    switch (byte_number)
    {
    case 1:
        comm_result = _packetHandler->write1ByteTxRx(_portHandler.get(), id, reg_address, static_cast<uint8_t>(value));
        break;
    case 2:
        comm_result = _packetHandler->write2ByteTxRx(_portHandler.get(), id, reg_address, static_cast<uint16_t>(value));
        break;
    case 4:
        comm_result = _packetHandler->write4ByteTxRx(_portHandler.get(), id, reg_address, static_cast<uint32_t>(value));
        break;
    default:
        printf("ERROR: Size param must be 1, 2 or 4 bytes\n");
        break;
    }

    return comm_result;
}

/**
 * @brief TtlTools::getRegister
 * @param id
 * @param reg_address
 * @param value
 * @param byte_number
 * @return
 */
int TtlTools::getRegister(uint8_t id, uint16_t reg_address, uint32_t &value, uint8_t byte_number)
{
    int comm_result = COMM_TX_FAIL;
    uint8_t error = 0;

    switch (byte_number)
    {
    case 1:
    {
        uint8_t read_data;
        comm_result = _packetHandler->read1ByteTxRx(_portHandler.get(), id, reg_address, &read_data, &error);
        value = read_data;
    }
    break;
    case 2:
    {
        uint16_t read_data;
        comm_result = _packetHandler->read2ByteTxRx(_portHandler.get(), id, reg_address, &read_data, &error);
        value = read_data;
    }
    break;
    case 4:
    {
        uint32_t read_data;
        comm_result = _packetHandler->read4ByteTxRx(_portHandler.get(), id, reg_address, &read_data, &error);
        value = read_data;
    }
    break;
    default:
        printf("ERROR: Size param must be 1, 2 or 4 bytes\n");
        break;
    }

    if (0 != error)
        comm_result = error;

    return comm_result;
}

/**
 * @brief TtlTools::setRegisters
 */
int TtlTools::setRegisters(std::vector<uint8_t> ids, uint16_t reg_address, std::vector<uint32_t> values, uint8_t byte_number)
{
    int dxl_comm_result = COMM_SUCCESS;

    if (!ids.empty())
    {
        if (ids.size() == values.size())
        {
            dynamixel::GroupSyncWrite groupSyncWrite(_portHandler.get(), _packetHandler.get(), reg_address, byte_number);

            bool dxl_senddata_result = false;

            for (size_t i = 0; i < ids.size(); ++i)
            {
                uint8_t id = ids.at(i);
                uint32_t data = values.at(i);

                switch (byte_number)
                {
                case 1:
                {
                    uint8_t params[1] = {static_cast<uint8_t>(data)};
                    dxl_senddata_result = groupSyncWrite.addParam(id, params);
                }
                break;
                case 2:
                {
                    uint8_t params[2] = {DXL_LOBYTE(static_cast<uint16_t>(data)), DXL_HIBYTE(static_cast<uint16_t>(data))};
                    dxl_senddata_result = groupSyncWrite.addParam(id, params);
                }
                break;
                case 4:
                {
                    uint8_t params[4] = {DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data))};
                    dxl_senddata_result = groupSyncWrite.addParam(id, params);
                }
                break;
                default:
                    printf("AbstractTtlDriver::syncWrite ERROR: Size param must be 1, 2 or 4 bytes\n");
                    break;
                }

                if (!dxl_senddata_result)
                {
                    dxl_comm_result = COMM_TX_ERROR;
                    break;
                }
            }

            // send group if no error
            if (COMM_TX_ERROR != dxl_comm_result)
                dxl_comm_result = groupSyncWrite.txPacket();

            groupSyncWrite.clearParam();
        }
        else
        {
            dxl_comm_result = COMM_TX_ERROR;
        }
    }

    return dxl_comm_result;
}

/**
 * @brief TtlTools::getRegisters
 */
int TtlTools::getRegisters(std::vector<uint8_t> ids, uint16_t reg_address, std::vector<uint32_t> &values, uint8_t byte_number)
{
    values.clear();

    dynamixel::GroupSyncRead groupSyncRead(_portHandler.get(), _packetHandler.get(), reg_address, byte_number);
    int dxl_comm_result = COMM_TX_FAIL;

    for (auto const &id : ids)
    {
        if (!groupSyncRead.addParam(id))
        {
            groupSyncRead.clearParam();
            return COMM_RX_FAIL;
        }
    }

    dxl_comm_result = groupSyncRead.txRxPacket();

    if (COMM_SUCCESS == dxl_comm_result)
    {
        for (auto const &id : ids)
        {
            if (groupSyncRead.isAvailable(id, reg_address, byte_number))
            {
                switch (byte_number)
                {
                case 1:
                    values.emplace_back(static_cast<uint8_t>(groupSyncRead.getData(id, reg_address, byte_number)));
                    break;
                case 2:
                    values.emplace_back(static_cast<uint16_t>(groupSyncRead.getData(id, reg_address, byte_number)));
                    break;
                case 4:
                    values.emplace_back(groupSyncRead.getData(id, reg_address, byte_number));
                    break;
                default:
                    printf("AbstractTtlDriver::syncRead ERROR: Size param must be 1, 2 or 4 bytes\n");
                    break;
                }
            }
            else
            {
                dxl_comm_result = COMM_RX_FAIL;
                break;
            }
        }
    }

    groupSyncRead.clearParam();

    return dxl_comm_result;
}

/**
 * @brief TtlTools::closePort
 */
void TtlTools::closePort() { _portHandler->closePort(); }
}  // namespace ttl_debug_tools
