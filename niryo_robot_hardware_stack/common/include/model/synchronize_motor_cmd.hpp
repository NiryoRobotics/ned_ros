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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DXL_SYNCHRONIZE_MOTOR_CMD_H
#define DXL_SYNCHRONIZE_MOTOR_CMD_H

#include <string>
#include <vector>
#include <set>
#include <memory>
#include <sstream>

#include "model/abstract_motor_cmd.hpp"
#include "model/dxl_command_type_enum.hpp"
#include "model/motor_type_enum.hpp"
#include "model/joint_state.hpp"

namespace common {
    namespace model {

        /**
         * @brief The SynchronizeMotorCmd class
         */
        class SynchronizeMotorCmd : public AbstractMotorCmd<EDxlCommandType>
        {
            struct MotorParam {
                MotorParam(uint8_t id, uint32_t param) {
                    motors_id.emplace_back(id);
                    params.emplace_back(param);
                }

                bool isValid() const {
                    return !motors_id.empty() && motors_id.size() == params.size();
                }

                std::vector<uint8_t> motors_id;
                std::vector<uint32_t> params;
            };

            public:
                SynchronizeMotorCmd();
                SynchronizeMotorCmd(EDxlCommandType type);

                //setters
                void addMotorParam(EMotorType type, uint8_t id, uint32_t param);

                //getters
                std::vector<uint8_t> getMotorsId(EMotorType type) const;
                std::vector<uint32_t> getParams(EMotorType type) const;
                std::set<EMotorType> getMotorTypes() const;

                // AbstractMotorCmd interface
                void reset() override;
                std::string str() const override;
                void clear() override;
                bool isValid() const override;

            private:
                std::set<EMotorType> _types;
                std::map<EMotorType, MotorParam > _motor_params_map;
        };

    } // namespace model
} // namespace common

#endif
