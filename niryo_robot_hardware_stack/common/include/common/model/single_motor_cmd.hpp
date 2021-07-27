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

#include "common/model/abstract_motor_cmd.hpp"
#include "common/model/dxl_command_type_enum.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/isingle_motor_cmd.hpp"

namespace common
{
namespace model
{

/**
 * @brief The SingleMotorCmd class
 */
template<typename E>
class SingleMotorCmd : public AbstractMotorCmd<E>, public ISingleMotorCmd
{
    public:
        SingleMotorCmd();
        SingleMotorCmd(E type);
        SingleMotorCmd(E type,
                       uint8_t motor_id,
                       uint32_t param = 0);

        SingleMotorCmd(E type,
                       uint8_t motor_id,
                       std::vector<int32_t> params = std::vector<int32_t>());

        // AbstractMotorCmd interface
        virtual void clear() override;


        // ISingleMotorCmd interface
        void setId(uint8_t id) override;
        void setParam(uint32_t param) override;
        void setParams(std::vector<int32_t> params) override;

        // getters
        uint8_t getId() const override;
        uint32_t getParam() const override;
        std::vector<int32_t> getParams() const override;

        // getters
        bool isCmdStepper() const override;
        bool isCmdDxl() const override;

        virtual int getTypeCmd() const override;

        // IObject interface
        virtual void reset() override;
        virtual std::string str() const override;
        virtual bool isValid() const override;

    private:
        uint8_t _id;
        uint32_t _param;
        std::vector<int32_t> _param_list;
};

/**
 * @brief SingleMotorCmd::SingleMotorCmd
 */
template<typename E>
SingleMotorCmd<E>::SingleMotorCmd() :
    AbstractMotorCmd<E>(E::CMD_TYPE_UNKNOWN)
{
    reset();
}

/**
 * @brief SingleMotorCmd::SingleMotorCmd
 * @param type
 */
template<typename E>
SingleMotorCmd<E>::SingleMotorCmd(E type) :
    AbstractMotorCmd<E>(type)
{
    clear();
}

/**
 * @brief SingleMotorCmd::SingleMotorCmd
 * @param type
 * @param motor_id
 * @param params
 */
template<typename E>
SingleMotorCmd<E>::SingleMotorCmd(E type,
                                  uint8_t motor_id,
                                  uint32_t param) :
    AbstractMotorCmd<E>(type),
    _id(motor_id),
    _param(param)
{
}

/**
 * @brief SingleMotorCmd::SingleMotorCmd
 * @param type
 * @param motor_id
 * @param param
 */
template<typename E>
SingleMotorCmd<E>::SingleMotorCmd(E type,
                                  uint8_t motor_id,
                                  std::vector<int32_t> params) :
    AbstractMotorCmd<E>(type),
    _id(motor_id),
    _param_list(params)
{}

// ***********************
//  AbstractMotorCmd intf
// ***********************

/**
 * @brief SingleMotorCmd<E>::setId
 * @param id
 */
template<typename E>
void SingleMotorCmd<E>::setId(uint8_t id)
{
    _id = id;
}

/**
 * @brief SingleMotorCmd<E>::setParam
 * @param param
 */
template<typename E>
void SingleMotorCmd<E>::setParam(uint32_t param)
{
    _param = param;
}

/**
 * @brief SingleMotorCmd<E>::setParam
 * @param param
 */
template<typename E>
void SingleMotorCmd<E>::setParams(std::vector<int32_t> params) 
{
    _param_list = params;
}

/**
 * @brief SingleMotorCmd<E>::getId
 * @return
 */
template<typename E>
uint8_t SingleMotorCmd<E>::getId() const
{
    return _id;
}

/**
 * @brief SingleMotorCmd<E>::getParam
 * @return
 */
template<typename E>
uint32_t SingleMotorCmd<E>::getParam() const
{
    return _param;
}

/**
 * @brief SingleMotorCmd<E>::getParams
 * @return
 */
template<typename E>
std::vector<int32_t> 
SingleMotorCmd<E>::getParams() const
{
    return _param_list;
}

/**
 * @brief SingleMotorCmd::getTypeCmd
 * @return
 */
template<typename E>
int SingleMotorCmd<E>::getTypeCmd() const
{
    return (int)this->getType();
}

/**
 * @brief SingleMotorCmd::isCmdStepper
 * @param none
 * @return
 */
template<typename E>
bool SingleMotorCmd<E>::isCmdStepper() const
{
    return typeid(E) == typeid(common::model::EStepperCommandType);
}

/**
 * @brief SingleMotorCmd::isCmdDxl
 * @param none
 * @return
 */
template<typename E>
bool SingleMotorCmd<E>::isCmdDxl() const
{
    return typeid(E) == typeid(common::model::EDxlCommandType);
}

/**
 * @brief SingleMotorCmd::reset
 */
template<typename E>
void SingleMotorCmd<E>::reset()
{
    this->setType(E::CMD_TYPE_UNKNOWN);
    clear();
}

/**
 * @brief SingleMotorCmd::clear
 */
template<typename E>
void SingleMotorCmd<E>::clear()
{
    _id = 0;
    _param = 0;
    _param_list.clear();
}


//********************************
// specializations for dynamixel
//********************************

template<>
inline
std::string SingleMotorCmd<EDxlCommandType>::str() const
{
    std::ostringstream ss;
    ss << "Dynamixel motor cmd - ";

    ss << DxlCommandTypeEnum(_type).toString() << " ";

    ss << "Motor id: ";
        ss << std::to_string(_id) << " ";

    ss << "; param: ";
    ss << std::to_string(_param);

    return ss.str();
}

/**
 * @brief SingleMotorCmd::isValid
 * @return
 */
template<>
inline
bool SingleMotorCmd<EDxlCommandType>::isValid() const
{
    return (EDxlCommandType::CMD_TYPE_UNKNOWN != this->getType()) &&
           (0 != _id);
}

//********************************
// specializations for steppers
//********************************

template<>
inline
std::string SingleMotorCmd<EStepperCommandType>::str() const
{
    std::ostringstream ss;
    ss << "Stepper motor cmd - ";

    ss << StepperCommandTypeEnum(_type).toString() << " ";

    ss << "Motor id: ";
        ss << std::to_string(_id) << " ";

    ss << "Params: ";
    for (int32_t param : getParams())
        ss << std::to_string(param) << " ";

    return ss.str();
}

template<>
inline
bool SingleMotorCmd<EStepperCommandType>::isValid() const
{
    if ((EStepperCommandType::CMD_TYPE_NONE == _type) ||
       (EStepperCommandType::CMD_TYPE_UNKNOWN == _type) ||
       (_id == 0) ||
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

using DxlSingleCmd = SingleMotorCmd<EDxlCommandType>;
using StepperSingleCmd = SingleMotorCmd<EStepperCommandType>;

} // namespace model
} // namespace common

#endif
