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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STEPPER_MOTOR_CMD_H
#define STEPPER_MOTOR_CMD_H

#include <string>
#include <vector>

#include "model/abstract_motor_cmd.hpp"
#include "model/stepper_command_type_enum.hpp"

namespace common {
    namespace model {

        class StepperMotorCmd : public AbstractMotorCmd<EStepperCommandType>
        {
            public:
                StepperMotorCmd();
                StepperMotorCmd(EStepperCommandType type);
                StepperMotorCmd(EStepperCommandType type,
                                uint8_t motor_id,
                                std::vector<int32_t> params = std::vector<int32_t>());

                //setters
                void setId(uint8_t id);
                void setParams(std::vector<int32_t> params);

                //getters
                uint8_t getId() const;
                const std::vector<int32_t>& getParams() const;

                // AbstractMotorCmd interface
                virtual void reset() override;
                virtual void clear() override;
                virtual std::string str() const override;
                virtual bool isValid() const override;

            private:
                uint8_t _id;
                std::vector<int32_t> _param_list;
        };

        inline
        uint8_t StepperMotorCmd::getId() const
        {
            return _id;
        }

        inline
        const std::vector<int32_t>& StepperMotorCmd::getParams() const
        {
            return _param_list;
        }
    } // namespace model
} // namespace common

#endif
