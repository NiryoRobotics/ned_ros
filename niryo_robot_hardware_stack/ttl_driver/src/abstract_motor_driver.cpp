/*
    abstract_motor_driver.cpp
    Copyright (C) 2020 Niryo
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

#include "ttl_driver/abstract_motor_driver.hpp"

#include <sstream>
#include <vector>
#include <string>

using ::std::shared_ptr;
using ::std::vector;
using ::std::string;
using ::std::ostringstream;

namespace ttl_driver
{
/**
 * @brief AbstractMotorDriver::AbstractMotorDriver
 * @param portHandler
 * @param packetHandler
 */
AbstractMotorDriver::AbstractMotorDriver(shared_ptr<dynamixel::PortHandler> portHandler,
                 shared_ptr<dynamixel::PacketHandler> packetHandler) :
    _dxlPortHandler(portHandler),
    _dxlPacketHandler(packetHandler)
{
}

/**
 * @brief AbstractMotorDriver::~AbstractMotorDriver
 */
AbstractMotorDriver::~AbstractMotorDriver()
{
}

/**
 * @brief AbstractMotorDriver::ping
 * @param id
 * @return
 */
int AbstractMotorDriver::ping(uint8_t id)
{
    uint8_t dxl_error = 0;

    int result = _dxlPacketHandler->ping(_dxlPortHandler.get(),
                                         id, &dxl_error);

    if (dxl_error != 0)
        result = dxl_error;

    return result;
}

/**
 * @brief AbstractMotorDriver::getModelNumber
 * @param id
 * @param dxl_model_number
 * @return
 */
int AbstractMotorDriver::getModelNumber(uint8_t id, uint16_t *dxl_model_number)
{
    uint8_t dxl_error = 0;

    int result = _dxlPacketHandler->ping(_dxlPortHandler.get(),
                                         id,
                                         dxl_model_number,
                                         &dxl_error);

    if (0 != dxl_error)
        result = dxl_error;

    return result;
}

/**
 * @brief AbstractMotorDriver::scan
 * @param id_list
 * @return
 */
int AbstractMotorDriver::scan(vector<uint8_t> &id_list)
{
    return _dxlPacketHandler->broadcastPing(_dxlPortHandler.get(), id_list);
}

/**
 * @brief AbstractMotorDriver::reboot
 * @param id
 * @return
 */
int AbstractMotorDriver::reboot(uint8_t id)
{
    int result = -1;

    uint8_t dxl_error = 0;
    result = _dxlPacketHandler->reboot(_dxlPortHandler.get(), id, &dxl_error);

    if (0 != dxl_error)
        result = dxl_error;

    return result;
}

/**
 * @brief AbstractMotorDriver::str : build a string describing the object. For debug purpose only
 * @return
 */
string AbstractMotorDriver::str() const
{
    ostringstream ss;

    ss << "packet handler " << (_dxlPacketHandler ? "OK" : "Not Ok") << "\n";
    ss << "port handler " << (_dxlPortHandler ? "OK" : "Not Ok") << "\n";

    return ss.str();
}

/*
 *  -----------------   Read Write operations   --------------------
 */

/**
 * @brief AbstractMotorDriver::read
 * @param address
 * @param data_len
 * @param id
 * @param data
 * @return
 */
int AbstractMotorDriver::read(uint8_t address, uint8_t data_len, uint8_t id, uint32_t *data)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    switch (data_len)
    {
        case DXL_LEN_ONE_BYTE:
        {
            uint8_t read_data;
            dxl_comm_result = _dxlPacketHandler->read1ByteTxRx(_dxlPortHandler.get(),
                                                               id, address, &read_data, &dxl_error);
            (*data) = read_data;
        }
        break;
        case DXL_LEN_TWO_BYTES:
        {
            uint16_t read_data;
            dxl_comm_result = _dxlPacketHandler->read2ByteTxRx(_dxlPortHandler.get(),
                                                               id, address, &read_data, &dxl_error);
            (*data) = read_data;
        }
        break;
        case DXL_LEN_FOUR_BYTES:
        {
            uint32_t read_data;
            dxl_comm_result = _dxlPacketHandler->read4ByteTxRx(_dxlPortHandler.get(),
                                                               id, address, &read_data, &dxl_error);
            (*data) = read_data;
        }
        break;
        default:
            printf("AbstractMotorDriver::read ERROR: Size param must be 1, 2 or 4 bytes\n");
        break;
    }

    if (0 != dxl_error)
        dxl_comm_result = dxl_error;

    return dxl_comm_result;
}


/**
 * @brief AbstractMotorDriver::write
 * @param address
 * @param data_len
 * @param id
 * @param data
 * @return
 */
int AbstractMotorDriver::write(uint8_t address, uint8_t data_len, uint8_t id, uint32_t data)
{
    int dxl_comm_result = COMM_TX_FAIL;

    switch (data_len)
    {
        case DXL_LEN_ONE_BYTE:
            dxl_comm_result = _dxlPacketHandler->write1ByteTxOnly(_dxlPortHandler.get(),
                                                                  id, address, static_cast<uint8_t>(data));
        break;
        case DXL_LEN_TWO_BYTES:
            dxl_comm_result = _dxlPacketHandler->write2ByteTxOnly(_dxlPortHandler.get(),
                                                                  id, address, static_cast<uint16_t>(data));
        break;
        case DXL_LEN_FOUR_BYTES:
            dxl_comm_result = _dxlPacketHandler->write4ByteTxOnly(_dxlPortHandler.get(),
                                                                  id, address, data);
        break;
        default:
            printf("AbstractMotorDriver::write ERROR: Size param must be 1, 2 or 4 bytes\n");
        break;
    }

    return dxl_comm_result;
}

/**
 * @brief AbstractMotorDriver::syncRead
 * @param address
 * @param data_len
 * @param id_list
 * @param data_list
 * @return
 */
int AbstractMotorDriver::syncRead(uint8_t address, uint8_t data_len,
                                  const vector<uint8_t> &id_list, vector<uint32_t> &data_list)
{
    data_list.clear();

    dynamixel::GroupSyncRead groupSyncRead(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, data_len);
    int dxl_comm_result = COMM_TX_FAIL;

    for (auto const& id : id_list)
    {
        if (!groupSyncRead.addParam(id))
        {
            groupSyncRead.clearParam();
            return GROUP_SYNC_REDONDANT_ID;
        }
    }

    dxl_comm_result = groupSyncRead.txRxPacket();

    if (COMM_SUCCESS == dxl_comm_result)
    {
        for (auto const& id : id_list)
        {
            if (groupSyncRead.isAvailable(id, address, data_len))
            {
                switch (data_len)
                {
                    case DXL_LEN_ONE_BYTE:
                        data_list.emplace_back(static_cast<uint8_t>(groupSyncRead.getData(id, address, data_len)));
                    break;
                    case DXL_LEN_TWO_BYTES:
                        data_list.emplace_back(static_cast<uint16_t>(groupSyncRead.getData(id, address, data_len)));
                    break;
                    case DXL_LEN_FOUR_BYTES:
                        data_list.emplace_back(groupSyncRead.getData(id, address, data_len));
                    break;
                    default:
                        printf("AbstractMotorDriver::syncRead ERROR: Size param must be 1, 2 or 4 bytes\n");
                    break;
                }
            }
            else
            {
                dxl_comm_result = GROUP_SYNC_READ_RX_FAIL;
                break;
            }
        }
    }

    groupSyncRead.clearParam();

    return dxl_comm_result;
}

/**
 * @brief AbstractMotorDriver::syncWrite
 * @param address
 * @param data_len
 * @param id_list
 * @param data_list
 * @return
 */
int AbstractMotorDriver::syncWrite(uint8_t address, uint8_t data_len,
                                   const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &data_list)
{
    int dxl_comm_result = COMM_SUCCESS;

    if (!id_list.empty())
    {
        if (id_list.size() == data_list.size())
        {
            dynamixel::GroupSyncWrite groupSyncWrite(_dxlPortHandler.get(),
                                                     _dxlPacketHandler.get(),
                                                     address,
                                                     data_len);

            bool dxl_senddata_result = false;

            for (size_t i = 0; i < id_list.size(); ++i)
            {
                uint8_t id = id_list.at(i);
                uint32_t data = data_list.at(i);

                switch (data_len)
                {
                    case DXL_LEN_ONE_BYTE:
                    {
                        uint8_t params[1] = {static_cast<uint8_t>(data)};
                        dxl_senddata_result = groupSyncWrite.addParam(id, params);
                    }
                    break;
                    case DXL_LEN_TWO_BYTES:
                    {
                        uint8_t params[2] =
                                            {DXL_LOBYTE(static_cast<uint16_t>(data)),
                                             DXL_HIBYTE(static_cast<uint16_t>(data))};
                        dxl_senddata_result = groupSyncWrite.addParam(id, params);
                    }
                    break;
                    case DXL_LEN_FOUR_BYTES:
                    {
                        uint8_t params[4] =
                                            {DXL_LOBYTE(DXL_LOWORD(data)),
                                             DXL_HIBYTE(DXL_LOWORD(data)),
                                             DXL_LOBYTE(DXL_HIWORD(data)),
                                             DXL_HIBYTE(DXL_HIWORD(data))};
                        dxl_senddata_result = groupSyncWrite.addParam(id, params);
                    }
                    break;
                    default:
                        printf("AbstractMotorDriver::syncWrite ERROR: Size param must be 1, 2 or 4 bytes\n");
                    break;
                }

                if (!dxl_senddata_result)
                {
                    dxl_comm_result = GROUP_SYNC_REDONDANT_ID;
                    break;
                }
            }

            // send group if no error
            if (GROUP_SYNC_REDONDANT_ID != dxl_comm_result)
                dxl_comm_result = groupSyncWrite.txPacket();

            groupSyncWrite.clearParam();
        }
        else
{
            dxl_comm_result = LEN_ID_DATA_NOT_SAME;
        }
    }

    return dxl_comm_result;
}

}  // namespace ttl_driver
