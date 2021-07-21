/*
synchronize_motor_cmd_interface.cpp
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

#include "common/model/synchronize_motor_cmd_interface.hpp"
#include <ros/ros.h>

namespace common
{
namespace model
{

SynchronizeMotorCmdI::SynchronizeMotorCmdI()
{
}

SynchronizeMotorCmdI::~SynchronizeMotorCmdI()
{
}

/**
 * @brief SynchronizeMotorCmdI::addMotorParam
 * @param type
 * @param id
 * @param param
 */
void SynchronizeMotorCmdI::addMotorParam(EMotorType type, uint8_t id, uint32_t param)
{
    // not yet in map
    if (!_motor_params_map.count(type))
    {
        _motor_params_map.insert(std::make_pair(type, MotorParam(id, param)));
        _types.insert(type);
    }
    else
    {
        _motor_params_map.at(type).motors_id.emplace_back(id);
        _motor_params_map.at(type).params.emplace_back(param);
    }
}

/**
 * @brief SynchronizeMotorCmdI::getMotorsId
 * @param type
 * @return
 */
std::vector<uint8_t> SynchronizeMotorCmdI::getMotorsId(EMotorType type) const
{
    if (!_motor_params_map.count(type))
        throw std::out_of_range("type not known of synchonized command");

    return _motor_params_map.at(type).motors_id;
}

/**
 * @brief SynchronizeMotorCmdI::getParams
 * @param type
 * @return
 */
std::vector<uint32_t> SynchronizeMotorCmdI::getParams(EMotorType type) const
{
    if (!_motor_params_map.count(type))
        throw std::out_of_range("type not known of synchonized command");

    return _motor_params_map.at(type).params;
}

/**
 * @brief SynchronizeMotorCmdI::getMotorTypes
 * @return
 */
std::set<EMotorType> SynchronizeMotorCmdI::getMotorTypes() const
{
    return _types;
}

/**
 * @brief SynchronizeMotorCmdI::isCmdStepper
*/
bool SynchronizeMotorCmdI::isCmdStepper() const
{
    return false;
}

/**
 * @brief SynchronizeMotorCmdI::isCmdDxl
*/
bool SynchronizeMotorCmdI::isCmdDxl() const
{
    return false;
}

/**
 * @brief SynchronizeMotorCmdI::isValid
*/
bool SynchronizeMotorCmdI::isValid() const
{
    return false;
}

/**
 * @brief SynchronizeMotorCmdI::str
*/
std::string SynchronizeMotorCmdI::str() const
{
    return "";
}

/**
 * @brief SynchronizeMotorCmdI::reset
*/
void SynchronizeMotorCmdI::reset()
{
}
}
}