/*
    abstract_end_effector_driver.cpp
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

#include "ttl_driver/abstract_end_effector_driver.hpp"

#include <sstream>
#include <vector>
#include <string>

using ::std::shared_ptr;
using ::std::string;
using ::std::ostringstream;

namespace ttl_driver
{

/**
 * @brief AbstractEndEffectorDriver::AbstractEndEffectorDriver
 */
AbstractEndEffectorDriver::AbstractEndEffectorDriver() :
  AbstractTtlDriver()
{
}

/**
 * @brief AbstractEndEffectorDriver::AbstractEndEffectorDriver
 * @param portHandler
 * @param packetHandler
 */
AbstractEndEffectorDriver::AbstractEndEffectorDriver(shared_ptr<dynamixel::PortHandler> portHandler,
                                                     shared_ptr<dynamixel::PacketHandler> packetHandler) :
  AbstractTtlDriver(portHandler, packetHandler)
{
}

/**
 * @brief AbstractEndEffectorDriver::~AbstractEndEffectorDriver
 */
AbstractEndEffectorDriver::~AbstractEndEffectorDriver()
{
}

/**
 * @brief AbstractEndEffectorDriver::str : build a string describing the object. For debug purpose only
 * @return
 */
std::string AbstractEndEffectorDriver::str() const
{
  return "AbstractEndEffectorDriver (" + AbstractTtlDriver::str() + ")";
}

}  // namespace ttl_driver
