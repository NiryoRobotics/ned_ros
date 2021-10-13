/*
abstract_synchronize_motor_cmd.hpp
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

#ifndef ABSTRACT_SYNCHRONIZE_MOTOR_CMD_H
#define ABSTRACT_SYNCHRONIZE_MOTOR_CMD_H

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <typeinfo>

#include "common/model/i_synchronize_motor_cmd.hpp"
#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/hardware_type_enum.hpp"
#include "common/model/joint_state.hpp"

namespace common
{
namespace model
{

/**
 * @brief The AbstractSynchronizeMotorCmd class
 */
template<typename ParamType>
class AbstractSynchronizeMotorCmd : public ISynchronizeMotorCmd
{
protected:
    struct MotorParam {
        MotorParam(uint8_t id, ParamType param) {
            motors_id.emplace_back(id);
            params.emplace_back(param);
        }

        bool isValid() const {
            return !motors_id.empty() && motors_id.size() == params.size();
        }

        std::vector<uint8_t> motors_id;
        std::vector<ParamType> params;
    };

public:
    AbstractSynchronizeMotorCmd() = default;
    ~AbstractSynchronizeMotorCmd() override = default;

    // test
    bool isValid() const override;

    // setters
    void clear();
    void addMotorParam(EHardwareType type, uint8_t id, ParamType param);

    // getters
    std::vector<uint8_t> getMotorsId(EHardwareType type) const;
    std::vector<ParamType> getParams(EHardwareType type) const;
    std::set<EHardwareType> getMotorTypes() const;

protected:
    std::set<EHardwareType> _motor_types;
    std::map<EHardwareType, MotorParam > _motor_params_map;

protected:
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
    AbstractSynchronizeMotorCmd( const AbstractSynchronizeMotorCmd& ) = default;
    AbstractSynchronizeMotorCmd( AbstractSynchronizeMotorCmd&& ) noexcept = default;

    AbstractSynchronizeMotorCmd& operator= ( AbstractSynchronizeMotorCmd && ) noexcept = default;
    AbstractSynchronizeMotorCmd& operator= ( const AbstractSynchronizeMotorCmd& ) = default;
};

/**
 * @brief AbstractSynchronizeMotorCmd<ParamType>::isValid
 * @return
 */
template<typename ParamType>
bool AbstractSynchronizeMotorCmd<ParamType>::isValid() const
{
    if (_motor_params_map.empty())
        return false;
    for (const auto& it : _motor_params_map)
    {
        if (it.second.isValid())
            continue;
        
        return false;
    }
    return true;
}

/**
 * @brief AbstractSynchronizeMotorCmd<ParamType>::clear
 */
template<typename ParamType>
void AbstractSynchronizeMotorCmd<ParamType>::clear()
{
    _motor_params_map.clear();
    _motor_types.clear();
}

/**
 * @brief AbstractSynchronizeMotorCmd::addMotorParam
 * @param type
 * @param id
 * @param param
 */
template<typename ParamType>
void AbstractSynchronizeMotorCmd<ParamType>::addMotorParam(EHardwareType type, uint8_t id, ParamType param)
{
    // not yet in map
    if (!_motor_params_map.count(type))
    {
        _motor_params_map.insert(std::make_pair(type, MotorParam(id, param)));
        _motor_types.insert(type);
    }
    else
    {
        _motor_params_map.at(type).motors_id.emplace_back(id);
        _motor_params_map.at(type).params.emplace_back(param);
    }
}

/**
 * @brief AbstractSynchronizeMotorCmd::getMotorsId
 * @param type
 * @return
 */
template<typename ParamType>
std::vector<uint8_t> 
AbstractSynchronizeMotorCmd<ParamType>::getMotorsId(EHardwareType type) const
{
    if (!_motor_params_map.count(type))
        throw std::out_of_range("type not known of synchonized command");

    return _motor_params_map.at(type).motors_id;
}

/**
 * @brief AbstractSynchronizeMotorCmd::getParams
 * @param type
 * @return
 */
template<typename ParamType>
std::vector<ParamType>
AbstractSynchronizeMotorCmd<ParamType>::getParams(EHardwareType type) const
{
    if (!_motor_params_map.count(type))
        throw std::out_of_range("type not known of synchonized command");

    return _motor_params_map.at(type).params;
}

/**
 * @brief AbstractSynchronizeMotorCmd::getMotorTypes
 * @return
 */
template<typename ParamType>
std::set<EHardwareType> 
AbstractSynchronizeMotorCmd<ParamType>::getMotorTypes() const
{
    return _motor_types;
}

using AbstractTtlSynchronizeMotorCmd = AbstractSynchronizeMotorCmd<uint32_t>;
using AbstractCanSynchronizeMotorCmd = AbstractSynchronizeMotorCmd<int32_t>;

} // namespace model
} // namespace common

#endif // ABSTRACT_SYNCHRONIZE_MOTOR_CMD_H
