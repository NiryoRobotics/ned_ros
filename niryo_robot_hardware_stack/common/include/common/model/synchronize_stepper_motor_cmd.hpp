/*
    synchronize_steper_motor_cmd.hpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SYNCHRONIZE_STEPPER_MOTOR_CMD_H
#define SYNCHRONIZE_STEPPER_MOTOR_CMD_H

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <sstream>

#include "common/model/abstract_motor_cmd.hpp"
#include "common/model/stepper_command_type_enum.hpp"
#include "common/model/motor_type_enum.hpp"
#include "common/model/joint_state.hpp"

namespace common {
    namespace model {

        /**
         * @brief The SynchronizeStepperMotorCmd class
         */
        class SynchronizeStepperMotorCmd : public AbstractMotorCmd<EStepperCommandType>
        {
            public:
                SynchronizeStepperMotorCmd();
                SynchronizeStepperMotorCmd(EStepperCommandType type);

                //setters
                void addMotorParam(const std::shared_ptr<JointState> &dxlState, int32_t param);
                void addMotorParam(const JointState &dxlState, int32_t param);

                //getters
                std::vector<uint8_t> getMotorsId() const;
                int32_t getParam(uint8_t motor_id) const;

                void clear() override;
                // AbstractMotorCmd interface
                void reset() override;
                std::string str() const override;
                bool isValid() const override;

            private:
                std::map<uint8_t, int32_t> _motor_params_map;
        };

    } // namespace model
} // namespace common

#endif
