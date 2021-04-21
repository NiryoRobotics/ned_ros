/*
    stepper_motor_state.cpp
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

#include "model/stepper_motor_state.hpp"
#include <sstream>
#include <cassert>

using namespace std;

namespace common {
    namespace model {
        StepperMotorState::StepperMotorState() :
            JointState ()
        {
        }

        StepperMotorState::StepperMotorState(uint8_t id) :
            JointState("unknown", EMotorType::MOTOR_TYPE_STEPPER, id),
            _last_time_read(0.0),
            _hw_fail_counter(0.0),
            _gear_ratio(0.0),
            _direction(0.0),
            _max_effort(0.0),
            _firmware_version("")
        {
        }

        StepperMotorState::StepperMotorState(string name, EMotorType type, uint8_t id) :
            JointState(name, type, id)
        {

        }

        StepperMotorState::~StepperMotorState()
        {

        }

        //****************
        //  Setters
        //****************

        void StepperMotorState::setLastTimeRead(double last_time)
        {
            _last_time_read = last_time;
        }

        void StepperMotorState::setHwFailCounter(double fail_counter)
        {
            _hw_fail_counter = fail_counter;
        }

        void StepperMotorState::setFirmwareVersion(string& firmware_version)
        {
            _firmware_version = firmware_version;
        }


        void StepperMotorState::setGearRatio(double gear_ratio)
        {
            _gear_ratio = gear_ratio;
        }
        
        void StepperMotorState::setDirection(double direction)
        {
            _direction = direction;
        }

        void StepperMotorState::setMaxEffort(double max_effort)
        {
            _max_effort = max_effort;
        }

        //*********************
        //  JointState Interface
        //********************

        void StepperMotorState::reset()
        {
            JointState::reset();
            _last_time_read = 0.0;
            _hw_fail_counter = 0.0;
            _firmware_version.clear();
        }

        bool StepperMotorState::isValid() const
        {
            return (0 != _id);
        }

        string StepperMotorState::str() const
        {
            ostringstream ss;
            
            ss << "StepperMotorState : ";
            ss << "\n---\n";
            ss << "last time read: " << _last_time_read << ", ";
            ss << "hw fail counter: " << _hw_fail_counter << ", ";
            ss << "firmware version: " << "\"" << _firmware_version << "\"";
            ss << "\n";
            ss << JointState::str();

            return ss.str();
        }

        int StepperMotorState::rad_pos_to_motor_pos(double pos_rad)
        {
            return ((STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * _gear_ratio * pos_rad * RADIAN_TO_DEGREE / 360.0) * _direction);
            //return (int32_t)((STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * gear_ratio * position_rad * RADIAN_TO_DEGREE / 360.0) * direction);
        }

        double StepperMotorState::to_rad_pos(int pos)
        {
            assert(0.0 != (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * _gear_ratio * RADIAN_TO_DEGREE));
            return static_cast<double>((static_cast<double>(pos) * 360.0) / (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * _gear_ratio * RADIAN_TO_DEGREE) * _direction);
            //return (double)((double)steps * 360.0 / (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * gear_ratio * RADIAN_TO_DEGREE)) * direction;
        }

        int StepperMotorState::stepsPerRev()
        {
            return int(STEPPERS_MICROSTEPS * STEPPERS_MOTOR_STEPS_PER_REVOLUTION);
        }

    } // namespace model
} // namespace common
