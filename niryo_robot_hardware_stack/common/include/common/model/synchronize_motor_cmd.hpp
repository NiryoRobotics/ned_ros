/*
synchronize_motor_cmd.hpp
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

#ifndef SYNCHRONIZE_MOTOR_CMD_H
#define SYNCHRONIZE_MOTOR_CMD_H

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <sstream>
#include <typeinfo>

#include "common/model/abstract_synchronize_motor_cmd.hpp"

#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/hardware_type_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The SynchronizeMotorCmd class
 */
template<typename E, typename ParamType>
class SynchronizeMotorCmd : public AbstractSynchronizeMotorCmd<ParamType>
{
    public:
        SynchronizeMotorCmd();
        SynchronizeMotorCmd(E type);

        // setters
        void setType(E type);

        // getters
        E getType() const;
        int getCmdType() const override;

        // AbstractSingleMotorCmd interface
        bool isStepperCmd() const override;
        bool isDxlCmd() const override;

        // IObject interface
        void reset() override;
        std::string str() const override;
        bool isValid() const override;

    private:
        E _type{E::CMD_TYPE_UNKNOWN};
};

/**
 * @brief SingleMotorCmd<E, ParamType>::SingleMotorCmd
 */
template<typename E, typename ParamType>
SynchronizeMotorCmd<E, ParamType>::SynchronizeMotorCmd() :
    SynchronizeMotorCmd<E, ParamType>(E::CMD_TYPE_UNKNOWN)
{}

/**
 * @brief SynchronizeMotorCmd<E, ParamType>::SynchronizeMotorCmd
 * @param type
 */
template<typename E, typename ParamType>
SynchronizeMotorCmd<E, ParamType>::SynchronizeMotorCmd(E type) :
    AbstractSynchronizeMotorCmd<ParamType>()
{
    static_assert(std::is_enum<E>::value, "E must be an enum");

    this->setType(type);
    this->clear();
}

/**
 * @brief SynchronizeMotorCmd<E, ParamType>::setType
 * @param type
 */
template<typename E, typename ParamType>
void SynchronizeMotorCmd<E, ParamType>::setType(E type)
{
    _type = type;
}

/**
 * @brief SynchronizeMotorCmd<E, ParamType>::getType
 * @return
 */
template<typename E, typename ParamType>
E SynchronizeMotorCmd<E, ParamType>::getType() const
{
    return _type;
}

/**
 * @brief SynchronizeMotorCmd<E, ParamType>::getCmdType
 * @return
 */
template<typename E, typename ParamType>
int SynchronizeMotorCmd<E, ParamType>::getCmdType() const
{
  return static_cast<int>(_type);
}

/**
 * @brief SynchronizeMotorCmd<E, ParamType>::isCmdStepper
 * @return
 */
template<typename E, typename ParamType>
bool SynchronizeMotorCmd<E, ParamType>::isStepperCmd() const
{
    return typeid(E) == typeid(common::model::EStepperCommandType);
}

/**
 * @brief SynchronizeMotorCmd<E, ParamType>::isCmdDxl
 * @return
 */
template<typename E, typename ParamType>
bool SynchronizeMotorCmd<E, ParamType>::isDxlCmd() const
{
    return typeid(E) == typeid(common::model::EDxlCommandType);
}

/**
 * @brief SynchronizeMotorCmd<E, ParamType>::reset
 */
template<typename E, typename ParamType>
void SynchronizeMotorCmd<E, ParamType>::reset()
{
    this->setType(E::CMD_TYPE_UNKNOWN);
    this->clear();
}

/**
 * @brief SynchronizeMotorCmd<E, ParamType>::isValid
 * @return
 */
template<typename E, typename ParamType>
bool SynchronizeMotorCmd<E, ParamType>::isValid() const
{
    if (E::CMD_TYPE_UNKNOWN == getType() || this->_motor_params_map.empty())
    {
        return false;
    }

    for (auto const& it_map : this->_motor_params_map)
    {
        if (!it_map.second.isValid())
            return false;
    }

    return true;
}

// aliases for all kind of motors
using DxlSyncCmd = SynchronizeMotorCmd<EDxlCommandType, uint32_t>;
using StepperTtlSyncCmd = SynchronizeMotorCmd<EStepperCommandType, uint32_t>;
using StepperSyncCmd = SynchronizeMotorCmd<EStepperCommandType, int32_t>;


//********************************
// specializations for dynamixel
//********************************

/**
 * @brief DxlSyncCmd::str
 * @return
 */
template<>
inline
std::string DxlSyncCmd::str() const
{
    std::string string_info;

    std::ostringstream ss;
    ss << "Dynamixel Sync motor cmd - ";
    ss << DxlCommandTypeEnum(_type).toString();
    ss << ": ";

    if (!isValid())
    {
        ss << "Corrupted command : invalid sync command ";
        string_info = ss.str();
    }
    else
    {
        ss << "[";

        for (auto const& param : _motor_params_map)
        {
            ss << HardwareTypeEnum(param.first).toString() << " => ";
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

//********************************
// specializations for steppers TTL
//********************************

/**
 * @brief StepperTtlSyncCmd::str
 * @return
 */
template<>
inline
std::string StepperTtlSyncCmd::str() const
{
    std::string string_info;

    std::ostringstream ss;
    ss << "Stepper TTL Sync motor cmd - ";
    ss << StepperCommandTypeEnum(_type).toString();
    ss << ": ";

    if (!isValid())
    {
        ss << "Corrupted command : invalid sync command ";
        string_info = ss.str();
    }
    else
    {
        ss << "[";

        for (auto const& param : _motor_params_map)
        {
            ss << HardwareTypeEnum(param.first).toString() << " => ";
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



//********************************
// specializations for steppers CAN
//********************************

/**
 * @brief StepperSyncCmd::str
 * @return
 */
template<>
inline
std::string StepperSyncCmd::str() const
{
    std::string string_info;

    std::ostringstream ss;
    ss << "Stepper CAN Sync motor cmd - ";
    ss << StepperCommandTypeEnum(_type).toString();
    ss << ": ";

    if (!isValid())
    {
        ss << "Corrupted command : invalid sync command ";
        string_info = ss.str();
    }
    else
    {
        ss << "[";

        for (auto const& param : _motor_params_map)
        {
            ss << HardwareTypeEnum(param.first).toString() << " => ";
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



} // namespace model
} // namespace common

#endif // SYNCHRONIZE_MOTOR_CMD_H
