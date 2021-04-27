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
                DxlMotorState(EMotorType type, uint8_t id, bool isTool = false);
                DxlMotorState(std::string name, EMotorType type, uint8_t id , bool isTool = false);

                virtual ~DxlMotorState() override;

                //getters
                bool isTool() const;

                // JointState interface
                virtual std::string str() const override;
                virtual void reset() override;
                virtual bool isValid() const override;

                virtual int to_motor_pos(double pos_rad) override;
                virtual double to_rad_pos(int position_dxl) override;

                uint32_t getPGain() const;
                void setPGain(uint32_t getPGain);

                uint32_t getIGain() const;
                void setIGain(uint32_t getIGain);

                uint32_t getDGain() const;
                void setDGain(uint32_t getDGain);

                uint32_t getFF1Gain() const;
                void setFF1Gain(uint32_t getFF1Gain);

                uint32_t getFF2Gain() const;
                void setFF2Gain(uint32_t value);

        protected:
                bool _isTool;

                uint32_t _p_gain;
                uint32_t _i_gain;
                uint32_t _d_gain;
                uint32_t _ff1_gain;
                uint32_t _ff2_gain;
        };

        inline
        bool DxlMotorState::isTool() const
        {
            return _isTool;
        }

    } // namespace model
} // namespace common

#endif
