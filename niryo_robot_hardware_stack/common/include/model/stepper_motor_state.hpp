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
#include "joint_state.hpp"

namespace common {
    namespace model {

        class StepperMotorState : public JointState
        {

            public:
                StepperMotorState();
                StepperMotorState(uint8_t id);
                StepperMotorState(std::string name, EMotorType type, uint8_t id );

                virtual ~StepperMotorState() override;

                void setLastTimeRead(double last_time);
                void setHwFailCounter(double fail_counter);
                void setFirmwareVersion(std::string& firmware_version);

                double getLastTimeRead() const;
                double getHwFailCounter() const;
                std::string getFirmwareVersion() const;

                double getGearRatio() const;
                void setGearRatio(double gear_ratio);

                double getDirection() const;
                void setDirection(double direction);

                double getMaxEffort() const;
                void setMaxEffort(double max_effort);

                static int stepsPerRev();

                // JointState interface
            public:
                virtual void reset() override;
                virtual bool isValid() const override;
                virtual std::string str() const override;

                virtual int to_motor_pos(double pos_rad) override;
                virtual double to_rad_pos(int pos) override;

            protected:
                double _last_time_read;
                double _hw_fail_counter;

                double _gear_ratio;
                double _direction;
                double _max_effort;

                std::string _firmware_version;

            private:

                static constexpr double STEPPERS_MICROSTEPS                 = 8.0;
                static constexpr double STEPPERS_MOTOR_STEPS_PER_REVOLUTION = 200.0;

        };

        inline
        double StepperMotorState::getMaxEffort() const
        {
            return _max_effort;
        }

        inline
        double StepperMotorState::getGearRatio() const
        {
            return _gear_ratio;
        }

        inline
        double StepperMotorState::getDirection() const
        {
            return _direction;
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
