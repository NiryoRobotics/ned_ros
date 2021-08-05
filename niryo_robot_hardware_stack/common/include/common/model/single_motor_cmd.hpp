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

#ifndef DXL_SINGLE_MOTOR_CMD_H
#define DXL_SINGLE_MOTOR_CMD_H

#include <string>
#include <vector>
#include <sstream>

#include "common/model/dxl_command_type_enum.hpp"
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

        virtual ~SingleMotorCmd() override;

        // setters
        void setType(E type);

        // getters
        E getType() const;
        int getCmdType() const override
        {
            return static_cast<int>(_type);
        }

        // AbstractSingleMotorCmd interface
        virtual bool isStepperCmd() const override;
        virtual bool isDxlCmd() const override;

        // IObject interface
        virtual void reset() override;
        virtual std::string str() const override;
        virtual bool isValid() const override;

    private:
        E _type{E::CMD_TYPE_UNKNOWN};

};

/**
 * @brief SingleMotorCmd<E, ParamType>::SingleMotorCmd
 */
template<typename E, typename ParamType>
SingleMotorCmd<E, ParamType>::SingleMotorCmd() :
    AbstractSingleMotorCmd<ParamType>(0)
{
    static_assert(std::is_enum<E>::value, "E must be an enum");
}

/**
 * @brief SingleMotorCmd<E, ParamType>::SingleMotorCmd
 * @param type
 */
template<typename E, typename ParamType>
SingleMotorCmd<E, ParamType>::SingleMotorCmd(E type) :
    SingleMotorCmd()
{
    static_assert(std::is_enum<E>::value, "E must be an enum");

    this->setType(type);
}

/**
 * @brief SingleMotorCmd<E, ParamType>::SingleMotorCmd
 * @param type
 * @param motor_id
 */
template<typename E, typename ParamType>
SingleMotorCmd<E, ParamType>::SingleMotorCmd(E type,
                                             uint8_t motor_id) :
    AbstractSingleMotorCmd<ParamType>(motor_id)
{
    static_assert(std::is_enum<E>::value, "E must be an enum");

    this->setType(type);
}

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

template<typename E, typename ParamType>
SingleMotorCmd<E, ParamType>::~SingleMotorCmd()
{}

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
 * @brief SingleMotorCmd<EDxlCommandType, uint32_t>::str
 * @return
 */
template<>
inline
std::string SingleMotorCmd<EDxlCommandType, uint32_t>::str() const
{
    std::ostringstream ss;
    ss << "Dynamixel motor cmd - ";

    ss << DxlCommandTypeEnum(_type).toString() << " ";

    ss << "Motor id: ";
        ss << std::to_string(_id) << " ";

    if(!_param_list.empty())
    {
        ss << "; param: ";
        ss << std::to_string(getParam());
    }

    return ss.str();
}

/**
 * @brief SingleMotorCmd<EDxlCommandType, uint32_t>::isValid
 * @return
 */
template<>
inline
bool SingleMotorCmd<EDxlCommandType, uint32_t>::isValid() const
{
    return (EDxlCommandType::CMD_TYPE_UNKNOWN != _type) &&
           (0 != _id);
}

//********************************
// specializations for steppers
//********************************

/**
 * @brief SingleMotorCmd<EStepperCommandType, int32_t>::str
 * @return
 */
template<>
inline
std::string SingleMotorCmd<EStepperCommandType, int32_t>::str() const
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
 * @brief SingleMotorCmd<EStepperCommandType, int32_t>::isValid
 * @return
 */
template<>
inline
bool SingleMotorCmd<EStepperCommandType, int32_t>::isValid() const
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
            return (_param_list.size() == 4);
        case EStepperCommandType::CMD_TYPE_CONVEYOR:
            return (_param_list.size() == 3);
        default:
            return (_param_list.size() == 1);
    }
}

// using for simplified usage

using DxlSingleCmd = SingleMotorCmd<EDxlCommandType, uint32_t>;
using StepperTtlSingleCmd = SingleMotorCmd<EStepperCommandType, uint32_t>;
using StepperSingleCmd = SingleMotorCmd<EStepperCommandType, int32_t>;

} // namespace model
} // namespace common

#endif
