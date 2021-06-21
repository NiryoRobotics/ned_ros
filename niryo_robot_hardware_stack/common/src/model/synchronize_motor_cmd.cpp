/*
    synchronize_motor_cmd.cpp
    Copyright (C) 2017 Niryo
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

#include "common/model/synchronize_motor_cmd.hpp"

// c++
#include <sstream>
#include <string>
#include <vector>
#include <set>

namespace common
{
namespace model
{

/**
 * @brief SynchronizeMotorCmd::SynchronizeMotorCmd
 */
SynchronizeMotorCmd::SynchronizeMotorCmd() :
    AbstractMotorCmd<EDxlCommandType>(EDxlCommandType::CMD_TYPE_UNKNOWN)

{
    reset();
}

/**
 * @brief SynchronizeMotorCmd::SynchronizeMotorCmd
 * @param type
 */
SynchronizeMotorCmd::SynchronizeMotorCmd(EDxlCommandType type) :
    AbstractMotorCmd<EDxlCommandType>(type)
{
}

/**
 * @brief SynchronizeMotorCmd::addMotorParam
 * @param type
 * @param id
 * @param param
 */
void SynchronizeMotorCmd::addMotorParam(EMotorType type, uint8_t id, uint32_t param)
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

// ***********************
//  Getters
// ***********************

/**
 * @brief SynchronizeMotorCmd::getMotorsId
 * @param type
 * @return
 */
std::vector<uint8_t>
SynchronizeMotorCmd::getMotorsId(EMotorType type) const
{
    if (!_motor_params_map.count(type))
        throw std::out_of_range("type not known of synchonized command");

    return _motor_params_map.at(type).motors_id;
}

/**
 * @brief SynchronizeMotorCmd::getParams
 * @param type
 * @return
 */
std::vector<uint32_t>
SynchronizeMotorCmd::getParams(EMotorType type) const
{
    if (!_motor_params_map.count(type))
        throw std::out_of_range("type not known of synchonized command");

    return _motor_params_map.at(type).params;
}

/**
 * @brief SynchronizeMotorCmd::getMotorTypes
 * @return
 */
std::set<EMotorType> SynchronizeMotorCmd::getMotorTypes() const
{
    return _types;
}

// ***********************
//  AbstractMotorCmd intf
// ***********************

/**
 * @brief SynchronizeMotorCmd::reset
 */
void SynchronizeMotorCmd::reset()
{
    setType(EDxlCommandType::CMD_TYPE_UNKNOWN);
    clear();
}

/**
 * @brief SynchronizeMotorCmd::str
 * @return
 */
std::string SynchronizeMotorCmd::str() const
{
    std::string string_info;

    std::ostringstream ss;
    ss << "Sync motor cmd - ";
    ss << DxlCommandTypeEnum(_type).toString();
    ss << ": ";

    if (!isValid())
    {
        ss << "Corrupted command : motors id list and params list size mismatch ";
        string_info = ss.str();
    }
    else
    {
        ss << "[";

        for (auto const& param : _motor_params_map)
        {
            ss << MotorTypeEnum(param.first).toString() << " => ";
            MotorParam p = param.second;
            for (size_t i = 0; i < p.motors_id.size() && i < p.params.size(); ++i)
                ss << "(" << static_cast<int>(p.motors_id.at(i)) << ", " << p.params.at(i) << ")" << ",";
        }

        string_info = ss.str();
        string_info.pop_back();

        string_info += "]";
    }

    return string_info;
}

/**
 * @brief SynchronizeMotorCmd::isValid
 * @return
 */
bool SynchronizeMotorCmd::isValid() const
{
    if (EDxlCommandType::CMD_TYPE_UNKNOWN == _type || _motor_params_map.empty())
        return false;

    for (auto const& it_map : _motor_params_map)
    {
        if (!it_map.second.isValid())
            return false;
    }

    return true;
}

/**
 * @brief SynchronizeMotorCmd::clear : clears the data (keep the cmd type)
 */
void SynchronizeMotorCmd::clear()
{
    _motor_params_map.clear();
    _types.clear();
}

}  // namespace model
}  // namespace common