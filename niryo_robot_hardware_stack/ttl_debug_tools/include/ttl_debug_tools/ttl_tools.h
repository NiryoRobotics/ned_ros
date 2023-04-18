/*
ttl_tools.h
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

#ifndef TTL_DEBUG_TOOLS_TTL_TOOLS_H
#define TTL_DEBUG_TOOLS_TTL_TOOLS_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace ttl_debug_tools
{

/**
 * @brief The TtlTools class
 */
class TtlTools
{
  public:
    TtlTools();
    TtlTools(std::shared_ptr<dynamixel::PortHandler> portHandler, std::shared_ptr<dynamixel::PacketHandler> packetHandler);

    int setupBus(int baudrate);
    void broadcastPing();
    void ping(int id);
    int setRegister(uint8_t id, uint16_t reg_address, uint32_t value, uint8_t byte_number);
    int getRegister(uint8_t id, uint16_t reg_address, uint32_t &value, uint8_t byte_number);

    int setRegisters(std::vector<uint8_t> ids, uint16_t reg_address, std::vector<uint32_t> values, uint8_t byte_number);
    int getRegisters(std::vector<uint8_t> ids, uint16_t reg_address, std::vector<uint32_t> &values, uint8_t byte_number);

    void closePort();

  protected:
    std::shared_ptr<dynamixel::PortHandler> _portHandler;
    std::shared_ptr<dynamixel::PacketHandler> _packetHandler;
};

}  // namespace ttl_debug_tools

#endif  // TTL_DEBUG_TOOLS_TTL_TOOLS_H
