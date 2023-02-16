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

#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using ::std::ostringstream;
using ::std::shared_ptr;
using ::std::string;
using ::std::vector;

namespace ttl_driver
{

/**
 * @brief AbstractTtlDriver::AbstractTtlDriver
 * @param portHandler
 * @param packetHandler
 */
AbstractTtlDriver::AbstractTtlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler, std::shared_ptr<dynamixel::PacketHandler> packetHandler)
    : _dxlPortHandler(std::move(portHandler)), _dxlPacketHandler(std::move(packetHandler))
{
}

/**
 * @brief AbstractTtlDriver::ping
 * @param id
 * @return
 */
int AbstractTtlDriver::ping(uint8_t id)
{
    uint8_t dxl_error = 0;

    int result = _dxlPacketHandler->ping(_dxlPortHandler.get(), id, &dxl_error);

    return result;
}

/**
 * @brief AbstractTtlDriver::getModelNumber
 * @param id
 * @param model_number
 * @return
 */
int AbstractTtlDriver::getModelNumber(uint8_t id, uint16_t &model_number)
{
    uint8_t dxl_error = 0;

    int result = _dxlPacketHandler->ping(_dxlPortHandler.get(), id, &model_number, &dxl_error);

    return result;
}

/**
 * @brief AbstractTtlDriver::scan
 * @param id_list
 * @return
 */
int AbstractTtlDriver::scan(vector<uint8_t> &id_list) { return _dxlPacketHandler->broadcastPing(_dxlPortHandler.get(), id_list); }

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
 * @brief AbstractTtlDriver::readCustom
 * @param address
 * @param data_len
 * @param id
 * @param data
 * @return
 */
int AbstractTtlDriver::readCustom(uint16_t address, uint8_t data_len, uint8_t id, uint32_t &data)
{
    // clean output data first
    data = 0;
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
 * @brief AbstractTtlDriver::writeCustom
 * @param address
 * @param data_len
 * @param id
 * @param data
 * @return
 */
int AbstractTtlDriver::writeCustom(uint16_t address, uint8_t data_len, uint8_t id, uint32_t data)
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t error = 0;

    switch (data_len)
    {
    case DXL_LEN_ONE_BYTE:
        dxl_comm_result = write<uint8_t>(address, id, static_cast<uint8_t>(data));
        break;
    case DXL_LEN_TWO_BYTES:
        dxl_comm_result = write<uint16_t>(address, id, static_cast<uint16_t>(data));
        break;
    case DXL_LEN_FOUR_BYTES:
        dxl_comm_result = write<uint32_t>(address, id, data);
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

}  // namespace ttl_driver
