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
#include "joint_state.hpp"

namespace common {
    namespace model {


        class DxlMotorState : public JointState
        {
            public:
                DxlMotorState();
                DxlMotorState(uint8_t id, EMotorType type, bool isTool = false);
                DxlMotorState(std::string name, EMotorType type, uint8_t id , bool isTool = false);

                virtual ~DxlMotorState() override;

                //getters
                bool isTool() const;

                // JointState interface
                virtual std::string str() const override;
                virtual void reset() override;
                virtual bool isValid() const override;

                virtual uint32_t rad_pos_to_motor_pos(double pos_rad) override;
                virtual double to_rad_pos() override;

                int getPGain() const;
                void setPGain(int getPGain);

                int getIGain() const;
                void setIGain(int getIGain);

                int getDGain() const;
                void setDGain(int getDGain);

                int getFF1Gain() const;
                void setFF1Gain(int getFF1Gain);

                int getFF2Gain() const;
                void setFF2Gain(int value);

        protected:
                bool _isTool;

                int _p_gain;
                int _i_gain;
                int _d_gain;
                int _ff1_gain;
                int _ff2_gain;
        };

        inline
        bool DxlMotorState::isTool() const
        {
            return _isTool;
        }

    } // namespace model
} // namespace common

#endif
