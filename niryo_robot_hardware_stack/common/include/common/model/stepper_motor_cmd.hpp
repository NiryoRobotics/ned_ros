/*
stepper_motor_cmd.hpp
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

#ifndef STEPPER_MOTOR_CMD_H
#define STEPPER_MOTOR_CMD_H

#include <string>
#include <vector>

#include "common/model/abstract_motor_cmd.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/single_motor_cmd_interface.hpp"
namespace common
{
namespace model
{

/**
 * @brief The StepperMotorCmd class
 */
class StepperMotorCmd : public AbstractMotorCmd<EStepperCommandType>, public ISingleMotorCmd
{
    public:
        StepperMotorCmd();
        StepperMotorCmd(EStepperCommandType type);
        StepperMotorCmd(EStepperCommandType type,
                        uint8_t motor_id,
                        std::vector<int32_t> params = std::vector<int32_t>());

        // AbstractMotorCmd interface
        // getters
        virtual int getTypeCmd() const override;

        bool isCmdStepper() const override;
        bool isCmdDxl() const override;
        virtual void reset() override;
        virtual void clear() override;
        virtual std::string str() const override;
        virtual bool isValid() const override;
};

/**
 * @brief SingleMotorCmd::getType
 * @return
 */
inline int StepperMotorCmd::getTypeCmd() const
{
    return (int)this->getType();
}

/**
 * @brief StepperMotorCmd::isCmdStepper
*/
inline bool StepperMotorCmd::isCmdStepper() const
{
    return true;
}

/**
 * @brief StepperMotorCmd::isCmdStepper
*/
inline bool StepperMotorCmd::isCmdDxl() const
{
    return false;
}

} // namespace model
} // namespace common

#endif
