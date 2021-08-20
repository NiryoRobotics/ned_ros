/*
ttl_motor_driver.hpp
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
 * @brief The XDriver class
 */
class AbstractTtlDriver
{

public:
    AbstractTtlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                      std::shared_ptr<dynamixel::PacketHandler> packetHandler);
    virtual ~AbstractTtlDriver();

    int ping(uint8_t id);
    int getModelNumber(uint8_t id,
                       uint16_t& dxl_model_number);
    int scan(std::vector<uint8_t>& id_list);
    int reboot(uint8_t id);

    virtual std::string str() const;

public:
    // here are only common TTL commands found in both Steppers and DXl
    virtual std::string interpreteErrorState(uint32_t hw_state) = 0;

    // eeprom write

    // eeprom read
    virtual int checkModelNumber(uint8_t id) = 0;
    virtual int readFirmwareVersion(uint8_t id, uint32_t& version) = 0;

    // ram read
    virtual int readTemperature(uint8_t id, uint32_t& temperature) = 0;
    virtual int readVoltage(uint8_t id, uint32_t& voltage) = 0;
    virtual int readHwErrorStatus(uint8_t id, uint32_t& hardware_status) = 0;

    virtual int syncReadTemperature(const std::vector<uint8_t>& id_list, std::vector<uint32_t>& temperature_list) = 0;
    virtual int syncReadVoltage(const std::vector<uint8_t>& id_list, std::vector<uint32_t>& voltage_list) = 0;
    virtual int syncReadHwErrorStatus(const std::vector<uint8_t>& id_list, std::vector<uint32_t>& hw_error_list) = 0;

    // we use those commands in the children classes to actually read and write values in registers
    int read(uint8_t address, uint8_t data_len, uint8_t id, uint32_t& data);
    int write(uint8_t address, uint8_t data_len, uint8_t id, uint32_t data);
protected:

    int syncRead(uint8_t address, uint8_t data_len, const std::vector<uint8_t>& id_list, std::vector<uint32_t>& data_list);
    int syncWrite(uint8_t address, uint8_t data_len, const std::vector<uint8_t>& id_list, const std::vector<uint32_t>& data_list);

    static constexpr int PING_WRONG_MODEL_NUMBER = 30;

private:
    std::shared_ptr<dynamixel::PortHandler> _dxlPortHandler;
    std::shared_ptr<dynamixel::PacketHandler> _dxlPacketHandler;

    static constexpr uint8_t DXL_LEN_ONE_BYTE    = 1;
    static constexpr uint8_t DXL_LEN_TWO_BYTES   = 2;
    static constexpr uint8_t DXL_LEN_FOUR_BYTES  = 4;

    static constexpr int GROUP_SYNC_REDONDANT_ID = 10;
    static constexpr int GROUP_SYNC_READ_RX_FAIL = 11;
    static constexpr int LEN_ID_DATA_NOT_SAME    = 20;
};

} // ttl_driver

#endif // ABSTRACT_TTL_DRIVER_HPP
