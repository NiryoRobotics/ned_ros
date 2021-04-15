/*
    stepper_motor_state.h
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

#ifndef STEPPER_MOTOR_STATE_H
#define STEPPER_MOTOR_STATE_H

#include <string>
#include "abstract_motor_state.hpp"

namespace common {
    namespace model {

        class StepperMotorState : public AbstractMotorState
        {

            public:
                StepperMotorState();
                StepperMotorState(uint8_t id);
                virtual ~StepperMotorState() override;

                void setLastTimeRead(double last_time);
                void setHwFailCounter(double fail_counter);
                void setFirmwareVersion(std::string& firmware_version);

                double getLastTimeRead() const;
                double getHwFailCounter() const;
                std::string getFirmwareVersion() const;

                virtual bool operator==(const StepperMotorState& other);

                // AbstractMotorState interface
                virtual std::string str() const override;
                virtual void reset() override;
                virtual bool isValid() const override;

            protected:
                double _last_time_read;
                double _hw_fail_counter;
                std::string _firmware_version;
        };

        inline
        bool StepperMotorState::isValid() const
        {
            return (0 != _id);
        }

        inline
        double StepperMotorState::getLastTimeRead() const
        {
            return _last_time_read;
        }

        inline
        double StepperMotorState::getHwFailCounter() const
        {
            return _hw_fail_counter;
        }

        inline
        std::string StepperMotorState::getFirmwareVersion() const
        {
            return _firmware_version;
        }

    } // model
} // common

#endif
