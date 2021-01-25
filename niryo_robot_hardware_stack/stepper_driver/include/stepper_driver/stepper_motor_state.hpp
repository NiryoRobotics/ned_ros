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
#include "stepper_driver/stepper_enum.hpp"

namespace StepperDriver
{
    class StepperMotorState {

        public:
        
            StepperMotorState(uint8_t id);

            uint8_t getId() const;
            void setId(uint8_t motor_id);

            int32_t getPositionState() const;
            void setPositionState(int32_t pos);

            int32_t getTemperatureState() const;
            void setTemperatureState(int32_t temp);

            int32_t getHardwareErrorState() const;
            void setHardwareError(int32_t hw_error);

            double getLastTimeRead() const;
            void setLastTimeRead(double last_time);

            int getHwFailCounter() const;
            void setHwFailCounter(double fail_counter);

            std::string getFirmwareVersion() const;
            void setFirmwareVersion(std::string& firmware_version);

            bool operator==(const StepperMotorState& other);

        private:

            uint8_t _id;
            int32_t _state_pos=0;
            int32_t _state_temperature=0;
            int32_t _hw_error=0;
            double _last_time_read = 0;
            double _hw_fail_counter;
            std::string _firmware_version;
    };
}

#endif
