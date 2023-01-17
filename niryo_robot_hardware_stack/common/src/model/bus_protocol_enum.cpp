/*
    but_protocol_enum.cpp
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

#include "common/model/bus_protocol_enum.hpp"
#include <map>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief BusProtocolEnum::BusProtocolEnum
 * @param e
 */
BusProtocolEnum::BusProtocolEnum(EBusProtocol e) : AbstractEnum<BusProtocolEnum, EBusProtocol>(e) {}

/**
 * @brief BusProtocolEnum::BusProtocolEnum
 * @param str
 */
BusProtocolEnum::BusProtocolEnum(const char *const str) : AbstractEnum<BusProtocolEnum, EBusProtocol>(str) {}

/**
 * @brief BusProtocolEnum::initialize
 * @return
 */
std::map<EBusProtocol, std::string> BusProtocolEnum::initialize()
{
    std::map<EBusProtocol, std::string> m;

    m[EBusProtocol::TTL] = "ttl";
    m[EBusProtocol::CAN] = "can";
    m[EBusProtocol::UNKNOWN] = "unknown";

    return m;
}

}  // namespace model
}  // namespace common
