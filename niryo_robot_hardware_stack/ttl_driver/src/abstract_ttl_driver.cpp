/*
    abstract_ttl_driver.cpp
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

#include "ttl_driver/abstract_ttl_driver.hpp"

#include <sstream>
#include <utility>
#include <vector>
#include <string>

using ::std::shared_ptr;
using ::std::vector;
using ::std::string;
using ::std::ostringstream;

namespace ttl_driver
{

/**
 * @brief AbstractTtlDriver::AbstractTtlDriver
 * @param portHandler
 * @param packetHandler
 */
AbstractTtlDriver::AbstractTtlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                     std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    _dxlPortHandler(std::move(portHandler)),
    _dxlPacketHandler(std::move(packetHandler))
{}

/**
 * @brief AbstractTtlDriver::ping
 * @param id
 * @return
 */
int AbstractTtlDriver::ping(uint8_t id)
{
    uint8_t dxl_error = 0;

    int result = _dxlPacketHandler->ping(_dxlPortHandler.get(),
                                         id, &dxl_error);

    return result;
}

/**
 * @brief AbstractTtlDriver::getModelNumber
 * @param id
 * @param model_number
 * @return
 */
int AbstractTtlDriver::getModelNumber(uint8_t id, uint16_t& model_number)
{
    uint8_t dxl_error = 0;

    int result = _dxlPacketHandler->ping(_dxlPortHandler.get(),
                                         id,
                                         &model_number,
                                         &dxl_error);

    return result;
}

/**
 * @brief AbstractTtlDriver::scan
 * @param id_list
 * @return
 */
int AbstractTtlDriver::scan(vector<uint8_t> &id_list)
{
    return _dxlPacketHandler->broadcastPing(_dxlPortHandler.get(), id_list);
}

/**
 * @brief AbstractTtlDriver::reboot
 * @param id
 * @return
 */
int AbstractTtlDriver::reboot(uint8_t id)
{
    int result = -1;
    uint8_t dxl_error = 0;

    result = _dxlPacketHandler->reboot(_dxlPortHandler.get(), id, &dxl_error);

    return result;
}

/**
 * @brief AbstractTtlDriver::readCustom
 * @param address
 * @param data_len
 * @param id
 * @param data
 * @return
 */
int AbstractTtlDriver::readCustom(uint16_t address, uint8_t data_len, uint8_t id, uint32_t &data)
{
    return read(address, data_len, id, data);
}

/**
 * @brief AbstractTtlDriver::writeCustom
 * @param address
 * @param data_len
 * @param id
 * @param data
 * @return
 */
int AbstractTtlDriver::writeCustom(uint16_t address, uint8_t data_len, uint8_t id, uint32_t data)
{
    return write(address, data_len, id, data);
}

/**
 * @brief AbstractTtlDriver::str : build a string describing the object. For debug purpose only
 * @return
 */
std::string AbstractTtlDriver::str() const
{
    ostringstream ss;

    ss << "TTL Driver : "
       << "packet handler " << (_dxlPacketHandler ? "OK" : "Not Ok") << ","
       << "port handler " << (_dxlPortHandler ? "OK" : "Not Ok");

    return ss.str();
}

/*
 *  -----------------   Read Write operations   --------------------
 */

/**
 * @brief AbstractTtlDriver::read
 * @param address
 * @param data_len
 * @param id
 * @param data
 * @return
 */
int AbstractTtlDriver::read(uint16_t address, uint8_t data_len, uint8_t id, uint32_t& data)
{
    // clean output data first
    data = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    switch (data_len)
    {
        case DXL_LEN_ONE_BYTE:
        {
            uint8_t read_data;
            dxl_comm_result = read<uint8_t>(address, id, read_data);
            data = read_data;
        }
        break;
        case DXL_LEN_TWO_BYTES:
        {
            uint16_t read_data;
            dxl_comm_result = read<uint16_t>(address, id, read_data);
            data = read_data;
        }
        break;
        case DXL_LEN_FOUR_BYTES:
        {
            uint32_t read_data;
            dxl_comm_result = read<uint32_t>(address, id, read_data);
            data = read_data;
        }
        break;
        default:
            printf("AbstractTtlDriver::read ERROR: Size param must be 1, 2 or 4 bytes\n");
        break;
    }

    return dxl_comm_result;
}

/**
 * @brief AbstractTtlDriver::write
 * @param address
 * @param data_len
 * @param id
 * @param data
 * @return
 */
int AbstractTtlDriver::write(uint16_t address, uint8_t data_len, uint8_t id, uint32_t data)
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t error = 0;

    switch (data_len)
    {
        case DXL_LEN_ONE_BYTE:
            dxl_comm_result = _dxlPacketHandler->write1ByteTxRx(_dxlPortHandler.get(),
                                                                  id, address, static_cast<uint8_t>(data), &error);
        break;
        case DXL_LEN_TWO_BYTES:
            dxl_comm_result = _dxlPacketHandler->write2ByteTxRx(_dxlPortHandler.get(),
                                                                  id, address, static_cast<uint16_t>(data), &error);
        break;
        case DXL_LEN_FOUR_BYTES:
            dxl_comm_result = _dxlPacketHandler->write4ByteTxRx(_dxlPortHandler.get(),
                                                                  id, address, data, &error);
        break;
        default:
            printf("AbstractTtlDriver::write ERROR: Size param must be 1, 2 or 4 bytes\n");
        break;
    }

    if (error != 0)
    {
        printf("AbstractTtlDriver::write ERROR: device return error: id=%d, addr=%d, len=%d, err=0x%02x\n", id, address, data_len, error);
    }

    return dxl_comm_result;
}

/**
 * @brief AbstractTtlDriver::syncRead
 * @param address
 * @param data_len
 * @param id_list
 * @param data_list
 * @return
 */
int AbstractTtlDriver::syncRead(uint8_t address, uint8_t data_len,
                                const std::vector<uint8_t> &id_list,
                                std::vector<uint32_t> &data_list)
{
    switch (data_len)
    {
      case DXL_LEN_ONE_BYTE:
        return syncRead_32<uint8_t>(address, id_list, data_list);
      case DXL_LEN_TWO_BYTES:
        return syncRead_32<uint16_t>(address, id_list, data_list);
      case DXL_LEN_FOUR_BYTES:
        return syncRead_32<uint32_t>(address, id_list, data_list);
      default:
        break;
    }

    printf("AbstractTtlDriver::syncRead ERROR: Size param must be 1, 2 or 4 bytes\n");
    return COMM_TX_FAIL;
}

/**
 * @brief AbstractTtlDriver::bulkRead
 * 
 * @param address 
 * @param id_list 
 * @param data_list 
 * @return int 
 */
int AbstractTtlDriver::bulkRead(std::vector<uint16_t> address,
                                uint8_t data_len,
                                const std::vector<uint8_t> &id_list,
                                std::vector<uint32_t> &data_list)
{
    int dxl_comm_result = COMM_TX_FAIL;

    if (data_len <= 4)
    {
        if (address.size() == id_list.size())
        {
            dynamixel::GroupBulkRead groupBulkRead(_dxlPortHandler.get(), _dxlPacketHandler.get());

            for (size_t i = 0; i < address.size(); i++)
            {
                if (!groupBulkRead.addParam(id_list[i], address[i], data_len))
                {
                    groupBulkRead.clearParam();
                    return GROUP_SYNC_REDONDANT_ID;
                }
            }

            dxl_comm_result = groupBulkRead.txRxPacket();

            if (COMM_SUCCESS == dxl_comm_result)
            {
                for (size_t i = 0; i < address.size(); i++)
                {
                    if (groupBulkRead.isAvailable(id_list[i], address[i], data_len))
                    {
                        uint32_t data = groupBulkRead.getData(id_list[i], address[i], data_len);
                        data_list.emplace_back(data);
                    }
                    else
                    {
                        dxl_comm_result = GROUP_SYNC_READ_RX_FAIL;
                        break;
                    }
                }
            }

            groupBulkRead.clearParam();
        }
        else
        {
            dxl_comm_result = LEN_ID_DATA_NOT_SAME;
            printf("AbstractTtlDriver::bulkRead ERROR: size of lists param must be equal\n");
        }
    }
    else
    {
        printf("AbstractTtlDriver::bulkRead ERROR: Size param must be 1, 2 or 4 bytes\n");
    }

    return dxl_comm_result;
}

/**
 * @brief AbstractTtlDriver::syncWrite
 * @param address
 * @param data_len
 * @param id_list
 * @param data_list
 * @return
 */
int AbstractTtlDriver::syncWrite(uint8_t address, uint8_t data_len,
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
                        printf("AbstractTtlDriver::syncWrite ERROR: Size param must be 1, 2 or 4 bytes\n");
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
