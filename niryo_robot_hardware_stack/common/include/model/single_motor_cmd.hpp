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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DXL_SINGLE_MOTOR_CMD_H
#define DXL_SINGLE_MOTOR_CMD_H

#include <string>
#include <vector>

#include "model/abstract_motor_cmd.hpp"
#include "model/dxl_command_type_enum.hpp"

namespace common {
    namespace model {

        class SingleMotorCmd : public AbstractMotorCmd<EDxlCommandType>
        {
            public:
                SingleMotorCmd();
                SingleMotorCmd(EDxlCommandType type,
                               uint8_t motor_id,
                               uint32_t param);

                //setters
                void setId(uint8_t id);
                void setParam(uint32_t param);

                //getters
                uint8_t getId() const;
                uint32_t getParam() const;

                // AbstractMotorCmd interface
                virtual void reset() override;
                virtual std::string str() const override;
                virtual bool isValid() const override;

            private:
                uint8_t _id;
                uint32_t _param;
        };

        inline
        uint8_t
        SingleMotorCmd::getId() const
        {
            return _id;
        }

        inline
        uint32_t
        SingleMotorCmd::getParam() const
        {
            return _param;
        }

    } // namespace model
} // namespace common

#endif
