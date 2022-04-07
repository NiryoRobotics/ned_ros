/*
abstract_ttl_driver.hpp
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

#ifndef ABSTRACT_TTL_DRIVER_HPP
#define ABSTRACT_TTL_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "common/common_defs.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "common/model/abstract_synchronize_motor_cmd.hpp"

namespace ttl_driver
{

/**
 * @brief The AbstractTtlDriver class
 */
class AbstractTtlDriver
{

public:
    AbstractTtlDriver() = default;
    AbstractTtlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                      std::shared_ptr<dynamixel::PacketHandler> packetHandler);
    virtual ~AbstractTtlDriver() = default;

    virtual int ping(uint8_t id);
    virtual int getModelNumber(uint8_t id,
                       uint16_t& model_number);
    virtual int scan(std::vector<uint8_t>& id_list);
    virtual int reboot(uint8_t id);

    virtual int readCustom(uint16_t address, uint8_t data_len, uint8_t id, uint32_t& data);
    virtual int writeCustom(uint16_t address, uint8_t data_len, uint8_t id, uint32_t data);
    
    virtual int writeSingleCmd(const std::unique_ptr<common::model::AbstractTtlSingleMotorCmd >& cmd) = 0;
    virtual int writeSyncCmd(int type, const std::vector<uint8_t>& ids, const std::vector<uint32_t>& params) = 0;

public:
    virtual std::string str() const;

    // here are only common TTL commands found in both Steppers and DXl
    virtual std::string interpretErrorState(uint32_t hw_state) const = 0;

    // eeprom write

    // eeprom read
    virtual int checkModelNumber(uint8_t id) = 0;
    virtual int readFirmwareVersion(uint8_t id, std::string& version) = 0;

    // ram read
    virtual int readTemperature(uint8_t id, uint8_t& temperature) = 0;
    virtual int readVoltage(uint8_t id, double& voltage) = 0;
    virtual int readHwErrorStatus(uint8_t id, uint8_t& hardware_error_status) = 0;

    virtual int syncReadFirmwareVersion(const std::vector<uint8_t>& id_list, std::vector<std::string>& firmware_version) = 0;
    virtual int syncReadTemperature(const std::vector<uint8_t>& id_list, std::vector<uint8_t>& temperature_list) = 0;
    virtual int syncReadVoltage(const std::vector<uint8_t>& id_list, std::vector<double>& voltage_list) = 0;
    virtual int syncReadRawVoltage(const std::vector<uint8_t>& id_list, std::vector<double>& voltage_list) = 0;
    virtual int syncReadHwErrorStatus(const std::vector<uint8_t>& id_list, std::vector<uint8_t>& hw_error_list) = 0;
    virtual int syncReadHwStatus(const std::vector<uint8_t> &id_list, std::vector<std::pair<double, uint8_t> >& data_array_list) = 0;

protected:
    // we use those commands in the children classes to actually read and write values in registers
    template<typename T>
    int read(uint16_t address, uint8_t id, T& data);

    template<typename T>
    int syncRead(uint16_t address, const std::vector<uint8_t>& id_list, std::vector<T>& data_list);

    template<typename T, const size_t N>
    int syncReadConsecutiveBytes(uint16_t address,
                                 const std::vector<uint8_t> &id_list,
                                 std::vector<std::array<T, N> >& data_list);

    template<typename T>
    int write(uint16_t address, uint8_t id, T data);

    template<typename T>
    int syncWrite(uint16_t address, const std::vector<uint8_t>& id_list, const std::vector<T>& data_list);

    static constexpr int PING_WRONG_MODEL_NUMBER = 30;

    virtual std::string interpretFirmwareVersion(uint32_t fw_version) const = 0;

private:
    std::shared_ptr<dynamixel::PortHandler> _dxlPortHandler;
    std::shared_ptr<dynamixel::PacketHandler> _dxlPacketHandler;

    static constexpr uint8_t DXL_LEN_ONE_BYTE    = 1;
    static constexpr uint8_t DXL_LEN_TWO_BYTES   = 2;
    static constexpr uint8_t DXL_LEN_FOUR_BYTES  = 4;

    static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
    static constexpr int GROUP_SYNC_READ_RX_FAIL = 11;
    static constexpr int LEN_ID_DATA_NOT_SAME    = 20;

protected:
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
    AbstractTtlDriver( const AbstractTtlDriver& ) = default;
    AbstractTtlDriver( AbstractTtlDriver&& ) = default;
    AbstractTtlDriver& operator= ( AbstractTtlDriver && ) = default;
    AbstractTtlDriver& operator= ( const AbstractTtlDriver& ) = default;
};

/**
 * @brief AbstractTtlDriver::read
 * @param address
 * @param id
 * @param data
 * @return
 * TODO(CC) : use a better method from dynamixel driver (bulk read ? maybe readRxTx ?)
 */
template<typename T>
int AbstractTtlDriver::read(uint16_t address, uint8_t id, T& data)
{
    // clean output data first
    data = 0;
    uint8_t error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    switch (sizeof(T))
    {
        case DXL_LEN_ONE_BYTE:
        {
            uint8_t raw_data;

            dxl_comm_result = _dxlPacketHandler->read1ByteTxRx(_dxlPortHandler.get(),
                                                               id, address, &raw_data, &error);
            data = static_cast<T>(raw_data);
        }
        break;
        case DXL_LEN_TWO_BYTES:
        {
            uint16_t raw_data;

            dxl_comm_result = _dxlPacketHandler->read2ByteTxRx(_dxlPortHandler.get(),
                                                               id, address, &raw_data, &error);
            data = static_cast<T>(raw_data);
        }
        break;
        case DXL_LEN_FOUR_BYTES:
        {
            uint32_t raw_data;

            dxl_comm_result = _dxlPacketHandler->read4ByteTxRx(_dxlPortHandler.get(),
                                                               id, address, &raw_data, &error);
            data = static_cast<T>(raw_data);
        }
        break;
        default:
            printf("AbstractTtlDriver::read ERROR: Size param must be 1, 2 or 4 bytes\n");
        break;
    }

    if (0 != error)
    {
        printf("AbstractTtlDriver::write ERROR: device return error: id=%d, addr=%d, len=%d, err=0x%02x\n", id, address, static_cast<int>(sizeof(T)), error);
        dxl_comm_result = error;
    }

    return dxl_comm_result;
}


/**
 * @brief AbstractTtlDriver::syncReadConsecutiveBytes
 * @param address
 * @param id_list
 * @param data_list
 * Reads N consecutive blocks of T bytes simultaneously
 * @return
 */
template<typename T, const size_t N>
int AbstractTtlDriver::syncReadConsecutiveBytes(uint16_t address,
                                                const std::vector<uint8_t> &id_list,
                                                std::vector<std::array<T, N> >& data_list)
{
    data_list.clear();
    uint16_t data_size = sizeof(T);
    int dxl_comm_result = COMM_TX_FAIL;

    dynamixel::GroupSyncRead groupSyncRead(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, data_size * N);

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
            if (groupSyncRead.isAvailable(id, address, data_size * N))
            {
                std::array<T, N> blocks{};

                for(uint8_t b = 0; b < N; ++b)
                {
                    T data = static_cast<T>(groupSyncRead.getData(id, address + b * data_size, data_size));
                    blocks.at(b) = data;
                }

                data_list.emplace_back(std::move(blocks));
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
 * @brief AbstractTtlDriver::syncRead
 * @param address
 * @param id_list
 * @param data_list
 * @return
 */
template<typename T>
int AbstractTtlDriver::syncRead(uint16_t address,
                                const std::vector<uint8_t> &id_list,
                                std::vector<T> &data_list)
{
    int dxl_comm_result = COMM_TX_FAIL;

    data_list.clear();
    uint8_t data_len = sizeof(T);
    if(data_len <= 4)
    {
        dynamixel::GroupSyncRead groupSyncRead(_dxlPortHandler.get(), _dxlPacketHandler.get(), address, data_len);

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
                    T data = static_cast<T>(groupSyncRead.getData(id, address, data_len));
                    data_list.emplace_back(data);
                }
                else
                {
                    dxl_comm_result = GROUP_SYNC_READ_RX_FAIL;
                    break;
                }
            }
        }

        groupSyncRead.clearParam();
    }
    else
    {
        printf("AbstractTtlDriver::syncRead ERROR: Size param must be 1, 2 or 4 bytes\n");
    }

    return dxl_comm_result;
}

template<typename T>
int AbstractTtlDriver::write(uint16_t address, uint8_t id, T data)
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t error = 0;

    switch (sizeof(T))
    {
        case DXL_LEN_ONE_BYTE:
            dxl_comm_result = _dxlPacketHandler->write1ByteTxRx(_dxlPortHandler.get(),
                                                                  id, address, data, &error);
        break;
        case DXL_LEN_TWO_BYTES:
            dxl_comm_result = _dxlPacketHandler->write2ByteTxRx(_dxlPortHandler.get(),
                                                                  id, address, data, &error);
        break;
        case DXL_LEN_FOUR_BYTES:
            dxl_comm_result = _dxlPacketHandler->write4ByteTxRx(_dxlPortHandler.get(),
                                                                  id, address, data, &error);
        break;
        default:
            printf("AbstractTtlDriver::write ERROR: Size param must be 1, 2 or 4 bytes\n");
        break;
    }

    if (0 != error)
    {
        printf("AbstractTtlDriver::write ERROR: device return error: id=%d, addr=%d, len=%d, err=0x%02x\n", id, address, static_cast<int>(sizeof(T)), error);
        dxl_comm_result = error;
    }

    return dxl_comm_result;
}

template<typename T>
int AbstractTtlDriver::syncWrite(uint16_t address, const std::vector<uint8_t>& id_list, const std::vector<T>& data_list)
{
    int dxl_comm_result = COMM_SUCCESS;
    uint8_t data_len = sizeof(T);

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
                T data = data_list.at(i);

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
                        uint8_t params[2] = {DXL_LOBYTE(static_cast<uint16_t>(data)),
                                             DXL_HIBYTE(static_cast<uint16_t>(data))};
                        dxl_senddata_result = groupSyncWrite.addParam(id, params);
                    }
                    break;
                    case DXL_LEN_FOUR_BYTES:
                    {
                          uint8_t params[4] = {DXL_LOBYTE(DXL_LOWORD(data)),
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

} // ttl_driver

#endif // ABSTRACT_TTL_DRIVER_HPP
