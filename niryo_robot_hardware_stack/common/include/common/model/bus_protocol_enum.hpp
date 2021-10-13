/*
    bus_protocol_enum.hpp
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

#ifndef BUS_PROTOCOL_ENUM_H
#define BUS_PROTOCOL_ENUM_H

#include <map>
#include <string>

#include "common/common_defs.hpp"
#include "abstract_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The EBusProtocol enum
 */
enum class EBusProtocol {
                        TTL,
                        CAN,
                        UNKNOWN
                      };

/**
 * @brief Specialization of AbstractEnum for Acknowledge status enum
 */
class BusProtocolEnum : public AbstractEnum<BusProtocolEnum, EBusProtocol>
{
public:
    BusProtocolEnum(EBusProtocol e=EBusProtocol::UNKNOWN);
    BusProtocolEnum(const char* str);

private:
    friend class AbstractEnum<BusProtocolEnum, EBusProtocol>;
    static std::map<EBusProtocol, std::string> initialize();
};

} // model
} // common

#endif // BUS_PROTOCOL_ENUM_H
