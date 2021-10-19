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

#ifndef CAN_DEBUG_TOOLS_CAN_TOOLS_H
#define CAN_DEBUG_TOOLS_CAN_TOOLS_H

#include <cstdint>
#include <string>
#include <memory>
#include <thread>
#include <vector>

#include "mcp_can_rpi/mcp_can_rpi.h"

namespace can_debug_tools
{

/**
 * @brief The CanTools class
 */
class CanTools
{
  static constexpr uint8_t MAX_MESSAGE_LENGTH = 8;

public:
    CanTools() = default;
    CanTools(std::shared_ptr<mcp_can_rpi::MCP_CAN> mcp_can);

    virtual ~CanTools();
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
    CanTools( const CanTools& ) = delete;
    CanTools( CanTools&& ) = delete;
    CanTools& operator= ( CanTools && ) = delete;
    CanTools& operator= ( const CanTools& ) = delete;

    int setupCommunication();

    void startDump(double check_data_freq = 0.1);

private:
    void controlLoop();

    std::string dumpData();
    uint8_t read(unsigned long *id, uint8_t *len, std::array<uint8_t, MAX_MESSAGE_LENGTH> &buf);

private:
    std::shared_ptr<mcp_can_rpi::MCP_CAN> _mcp_can;
    std::thread _control_loop_thread;

    bool _control_loop_ok{true};
    int _check_data_delay_ms{100};

};

}  // namespace can_debug_tools

#endif  // CAN_DEBUG_TOOLS_CAN_TOOLS_H
