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

#ifndef DXL_SYNCHRONIZE_MOTOR_CMD_H
#define DXL_SYNCHRONIZE_MOTOR_CMD_H

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <sstream>
#include <typeinfo>

#include "common/model/abstract_motor_cmd.hpp"
#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/motor_type_enum.hpp"
#include "common/model/joint_state.hpp"
#include "common/model/synchronize_motor_cmd_interface.hpp"

namespace common
{
namespace model
{

/**
 * @brief The SynchronizeMotorCmd class
 */
template<typename T, typename TE> 
class SynchronizeMotorCmd : public AbstractMotorCmd<T>, public ISynchronizeMotorCmd
{
    public:
        SynchronizeMotorCmd();
        SynchronizeMotorCmd(T type);

        // AbstractMotorCmd interface
        int getTypeCmd() const override;
        bool isCmdStepper() const override;
        bool isCmdDxl() const override;
        void reset() override;
        std::string str() const override;
        void clear() override;
        bool isValid() const override;
};

/**
 * @brief SynchronizeMotorCmd::SynchronizeMotorCmd
 */
template<typename T, typename TE>
SynchronizeMotorCmd<T, TE>::SynchronizeMotorCmd() :
    AbstractMotorCmd<T>(T::CMD_TYPE_UNKNOWN)

{
    reset();
}

/**
 * @brief SynchronizeMotorCmd::SynchronizeMotorCmd
 * @param type
 */
template<typename T, typename TE>
SynchronizeMotorCmd<T, TE>::SynchronizeMotorCmd(T type) :
    AbstractMotorCmd<T>(type)
{
}

// ***********************
//  AbstractMotorCmd intf
// ***********************

/**
 * @brief SynchronizeMotorCmd::reset
 */
template<typename T, typename TE>
void SynchronizeMotorCmd<T, TE>::reset()
{
    this->setType(T::CMD_TYPE_UNKNOWN);
    clear();
}

/**
 * @brief SynchronizeMotorCmd::str
 * @return
 */
template<typename T, typename TE>
std::string SynchronizeMotorCmd<T, TE>::str() const
{
    std::string string_info;

    std::ostringstream ss;
    ss << "Sync motor cmd - ";
    ss << TE(this->getType()).toString();
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
 * @brief SingleMotorCmd::getType
 * @return
 */
template<typename T, typename TE>
int SynchronizeMotorCmd<T, TE>::getTypeCmd() const
{
    return (int)this->getType();
}

/**
 * @brief SynchronizeMotorCmd::isValid
 * @return
 */
template<typename T, typename TE>
bool SynchronizeMotorCmd<T, TE>::isValid() const
{
    if (T::CMD_TYPE_UNKNOWN == this->getType() || _motor_params_map.empty())
    {
        return false;
    }

    for (auto const& it_map : _motor_params_map)
    {
        if (!it_map.second.isValid())
            return false;
    }

    return true;
}

/**
 * @brief SynchronizeMotorCmd::isCmdStepper
 * @param none
 * @return
 */
template<typename T, typename TE>
bool SynchronizeMotorCmd<T, TE>::isCmdStepper() const
{
    return typeid(T) == typeid(common::model::EStepperCommandType);
}

/**
 * @brief SynchronizeMotorCmd::isCmdDxl
 * @param none
 * @return
 */
template<typename T, typename TE>
bool SynchronizeMotorCmd<T, TE>::isCmdDxl() const
{
    return typeid(T) == typeid(common::model::EDxlCommandType);
}

/**
 * @brief SynchronizeMotorCmd::clear : clears the data (keep the cmd type)
 */
template<typename T, typename TE>
void SynchronizeMotorCmd<T, TE>::clear()
{
    _motor_params_map.clear();
    _types.clear();
}

using DxlSyncCmd = SynchronizeMotorCmd<EDxlCommandType, DxlCommandTypeEnum>;
using StepperSyncCmd = SynchronizeMotorCmd<EStepperCommandType, StepperCommandTypeEnum>;

} // namespace model
} // namespace common

#endif
