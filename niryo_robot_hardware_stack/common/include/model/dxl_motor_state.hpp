/*
    dxl_motor_state.h
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

#ifndef DXL_MOTOR_STATE_H
#define DXL_MOTOR_STATE_H

#include <string>
#include "abstract_motor_state.hpp"
#include "model/dxl_motor_type_enum.hpp"

namespace common {
    namespace model {


        class DxlMotorState : public AbstractMotorState
        {
            public:
                DxlMotorState();
                DxlMotorState(uint8_t id, EDxlMotorType type, bool isTool = false);

                virtual ~DxlMotorState() override;

                virtual void reset() override;
                virtual bool isValid() const override;

                //getters
                EDxlMotorType getType() const;
                bool isTool() const;

                //setters
                bool operator==(const DxlMotorState& other);

                virtual std::string str() const override;

            protected:
                EDxlMotorType _type;

                bool _isTool;
        };

        inline
        EDxlMotorType DxlMotorState::getType() const
        {
            return _type;
        }

        inline
        bool DxlMotorState::isTool() const
        {
            return _isTool;
        }

    } // namespace model
} // namespace common

#endif
