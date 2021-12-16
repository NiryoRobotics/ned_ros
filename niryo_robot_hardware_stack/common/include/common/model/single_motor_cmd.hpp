/*
single_motor_cmd.hpp
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

#ifndef SINGLE_MOTOR_CMD_H
#define SINGLE_MOTOR_CMD_H

#include <string>
#include <vector>
#include <sstream>

#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/end_effector_command_type_enum.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/abstract_single_motor_cmd.hpp"

namespace common
{
namespace model
{

/**
 * @brief The SingleMotorCmd class
 */
template<typename E, typename ParamType>
class SingleMotorCmd : public AbstractSingleMotorCmd<ParamType>
{
    public:
        SingleMotorCmd();
        SingleMotorCmd(E type);
        SingleMotorCmd(E type,
                       uint8_t motor_id);

        SingleMotorCmd(E type,
                       uint8_t motor_id,
                       std::vector<ParamType> params);

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

// using for simplified usage

using DxlSingleCmd = SingleMotorCmd<EDxlCommandType, uint32_t>;
using StepperTtlSingleCmd = SingleMotorCmd<EStepperCommandType, uint32_t>;
using EndEffectorSingleCmd = SingleMotorCmd<EEndEffectorCommandType, uint32_t>;

using StepperSingleCmd = SingleMotorCmd<EStepperCommandType, int32_t>;

//**********************************************

/**
 * @brief SingleMotorCmd<E, ParamType>::SingleMotorCmd
 */
template<typename E, typename ParamType>
SingleMotorCmd<E, ParamType>::SingleMotorCmd() :
  SingleMotorCmd<E, ParamType>::SingleMotorCmd(E::CMD_TYPE_UNKNOWN,
                                               1,
                                               std::vector<ParamType>())
{}

/**
 * @brief SingleMotorCmd<E, ParamType>::SingleMotorCmd
 * @param type
 */
template<typename E, typename ParamType>
SingleMotorCmd<E, ParamType>::SingleMotorCmd(E type) :
  SingleMotorCmd<E, ParamType>::SingleMotorCmd(type,
                                               1,
                                               std::vector<ParamType>())
{}

/**
 * @brief SingleMotorCmd<E, ParamType>::SingleMotorCmd
 * @param type
 * @param motor_id
 */
template<typename E, typename ParamType>
SingleMotorCmd<E, ParamType>::SingleMotorCmd(E type,
                                             uint8_t motor_id) :
  SingleMotorCmd<E, ParamType>::SingleMotorCmd(type,
                                               motor_id,
                                               std::vector<ParamType>())
{}

/**
 * @brief SingleMotorCmd<E, ParamType>::SingleMotorCmd
 * @param type
 * @param motor_id
 * @param params
 */
template<typename E, typename ParamType>
SingleMotorCmd<E, ParamType>::SingleMotorCmd(E type,
                                             uint8_t motor_id,
                                             std::vector<ParamType> params) :
    AbstractSingleMotorCmd<ParamType>(motor_id)
{
    static_assert(std::is_enum<E>::value, "E must be an enum");

    this->setType(type);
    this->setParams(params);
}

/**
 * @brief SingleMotorCmd<E, ParamType>::setType
 * @param type
 */
template<typename E, typename ParamType>
void SingleMotorCmd<E, ParamType>::setType(E type)
{
    _type = type;
}

/**
 * @brief SingleMotorCmd<E, ParamType>::getType
 * @return
 */
template<typename E, typename ParamType>
E SingleMotorCmd<E, ParamType>::getType() const
{
    return _type;
}

/**
 * @brief SingleMotorCmd<E, ParamType>::getCmdType
 * @return
 */
template<typename E, typename ParamType>
int SingleMotorCmd<E, ParamType>::getCmdType() const
{
  return static_cast<int>(_type);
}

/**
 * @brief SingleMotorCmd<E, ParamType>::isCmdStepper
 * @return
 */
template<typename E, typename ParamType>
bool SingleMotorCmd<E, ParamType>::isStepperCmd() const
{
    return typeid(E) == typeid(common::model::EStepperCommandType);
}

/**
 * @brief SingleMotorCmd<E, ParamType>::isCmdDxl
 * @return
 */
template<typename E, typename ParamType>
bool SingleMotorCmd<E, ParamType>::isDxlCmd() const
{
    return typeid(E) == typeid(common::model::EDxlCommandType);
}

/**
 * @brief SingleMotorCmd<E, ParamType>::reset
 */
template<typename E, typename ParamType>
void SingleMotorCmd<E, ParamType>::reset()
{
    this->setType(E::CMD_TYPE_UNKNOWN);
    this->clear();
}

//********************************
// specializations for dynamixel
//********************************

/**
 * @brief DxlSingleCmd::str
 * @return
 */
template<>
inline
std::string DxlSingleCmd::str() const
{
    std::ostringstream ss;
    ss << "Dynamixel motor cmd - ";

    ss << DxlCommandTypeEnum(_type).toString() << " ";

    ss << "Motor id: "
       << std::to_string(_id) << " "
       << "; param: ";

    for(auto const &p : _param_list)
    {
        ss << std::to_string(p) << " ";
    }

    return ss.str();
}

/**
 * @brief DxlSingleCmd::isValid
 * @return
 */
template<>
inline
bool DxlSingleCmd::isValid() const
{
    return (EDxlCommandType::CMD_TYPE_UNKNOWN != _type) &&
           (1 != _id) && (0 != _id);
}

//********************************
// specializations for steppers in TTL
//********************************

/**
 * @brief StepperTtlSingleCmd::str
 * @return
 */
template<>
inline
std::string StepperTtlSingleCmd::str() const
{
    std::ostringstream ss;
    ss << "Stepper motor cmd - ";

    ss << StepperCommandTypeEnum(_type).toString() << " ";

    ss << "Motor id: ";
        ss << std::to_string(_id) << " ";

    ss << "Params: ";
    for (auto param : getParams())
        ss << std::to_string(static_cast<uint32_t>(param)) << " ";

    return ss.str();
}

/**
 * @brief StepperTtlSingleCmd::isValid
 * @return
 */
template<>
inline
bool StepperTtlSingleCmd::isValid() const
{
    return !((EStepperCommandType::CMD_TYPE_NONE == getType()) ||
       (EStepperCommandType::CMD_TYPE_UNKNOWN == getType()) ||
       (getId() == 0));
}
//********************************
// specializations for steppers
//********************************

/**
 * @brief StepperSingleCmd::str
 * @return
 */
template<>
inline
std::string StepperSingleCmd::str() const
{
    std::ostringstream ss;
    ss << "Stepper motor cmd - ";

    ss << StepperCommandTypeEnum(_type).toString() << " ";

    ss << "Motor id: ";
        ss << std::to_string(_id) << " ";

    ss << "Params: ";
    for (int32_t param : getParams())
        ss << std::to_string(static_cast<int32_t>(param)) << " ";

    return ss.str();
}

/**
 * @brief StepperSingleCmd::isValid
 * @return
 */
template<>
inline
bool StepperSingleCmd::isValid() const
{
    if ((EStepperCommandType::CMD_TYPE_NONE == getType()) ||
       (EStepperCommandType::CMD_TYPE_UNKNOWN == getType()) ||
       (getId() == 0) ||
       (_param_list.empty()))
            return false;

    switch (_type)
    {
        case EStepperCommandType::CMD_TYPE_RELATIVE_MOVE:
            return (_param_list.size() == 2);
        case EStepperCommandType::CMD_TYPE_CALIBRATION:
            return (_param_list.size() == 4);
        case EStepperCommandType::CMD_TYPE_POSITION_OFFSET:
            return (_param_list.size() == 2);
        case EStepperCommandType::CMD_TYPE_CONVEYOR:
            return (_param_list.size() == 3);
        default:
            return (_param_list.size() == 1);
    }
}

//********************************
// specializations for end effector
//********************************

/**
 * @brief EndEffectorSingleCmd::str
 * @return
 */
template<>
inline
std::string EndEffectorSingleCmd::str() const
{
    std::ostringstream ss;
    ss << "End Effector cmd - ";

    ss << EndEffectorCommandTypeEnum(_type).toString() << " ";

    ss << "End Effector id: ";
        ss << std::to_string(_id) << " ";

    if(!_param_list.empty())
    {
        ss << "; param: ";
        ss << std::to_string(getParam());
    }

    return ss.str();
}

/**
 * @brief EndEffectorSingleCmd::isValid
 * @return
 */
template<>
inline
bool EndEffectorSingleCmd::isValid() const
{
    return (EEndEffectorCommandType::CMD_TYPE_UNKNOWN != _type) &&
           (1 != _id);
}

} // namespace model
} // namespace common

#endif // SINGLE_MOTOR_CMD_H
