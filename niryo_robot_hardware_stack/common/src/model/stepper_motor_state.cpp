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

#include "ros/time.h"

using namespace std;

namespace common {
    namespace model {

        /**
         * @brief StepperMotorState::StepperMotorState
         */
        StepperMotorState::StepperMotorState() :
            JointState (),
            _isConveyor(false)
        {
        }

        /**
         * @brief StepperMotorState::StepperMotorState
         * @param id
         * @param isConveyor
         */
        StepperMotorState::StepperMotorState(uint8_t id, bool isConveyor) :
            JointState("unknown", EMotorType::STEPPER, id),
            _isConveyor(isConveyor)
        {
        }

        /**
         * @brief StepperMotorState::StepperMotorState
         * @param name
         * @param type
         * @param id
         * @param isConveyor
         */
        StepperMotorState::StepperMotorState(std::string name, EMotorType type, uint8_t id, bool isConveyor) :
            JointState(name, type, id),
            _isConveyor(isConveyor)
        {

        }

        /**
         * @brief StepperMotorState::~StepperMotorState
         */
        StepperMotorState::~StepperMotorState()
        {

        }

        //****************
        //  Setters
        //****************

        /**
         * @brief StepperMotorState::updateLastTimeRead
         */
        void StepperMotorState::updateLastTimeRead()
        {
            _last_time_read = ros::Time::now().toSec();
        }

        /**
         * @brief StepperMotorState::setHwFailCounter
         * @param fail_counter
         */
        void StepperMotorState::setHwFailCounter(double fail_counter)
        {
            _hw_fail_counter = fail_counter;
        }

        /**
         * @brief StepperMotorState::setFirmwareVersion
         * @param firmware_version
         */
        void StepperMotorState::setFirmwareVersion(string& firmware_version)
        {
            _firmware_version = firmware_version;
        }

        /**
         * @brief StepperMotorState::setGearRatio
         * @param gear_ratio
         */
        void StepperMotorState::setGearRatio(double gear_ratio)
        {
            _gear_ratio = gear_ratio;
        }
        
        /**
         * @brief StepperMotorState::setDirection
         * @param direction
         */
        void StepperMotorState::setDirection(double direction)
        {
            _direction = direction;
        }

        /**
         * @brief StepperMotorState::setMaxEffort
         * @param max_effort
         */
        void StepperMotorState::setMaxEffort(double max_effort)
        {
            _max_effort = max_effort;
        }

        /**
         * @brief StepperMotorState::setCalibration
         * @param calibration_state
         * @param calibration_value
         */
        void StepperMotorState::setCalibration(const EStepperCalibrationStatus &calibration_state, const int32_t &calibration_value)
        {
            _calibration_state = calibration_state;
            _calibration_value = calibration_value;
        }

        //*********************
        //  JointState Interface
        //********************

        /**
         * @brief StepperMotorState::reset
         */
        void StepperMotorState::reset()
        {
            JointState::reset();
            _last_time_read = 0.0;
            _hw_fail_counter = 0.0;
            _firmware_version.clear();
            _calibration_value = 0;
            _calibration_state = EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED;
        }

        /**
         * @brief StepperMotorState::isValid
         * @return
         */
        bool StepperMotorState::isValid() const
        {
            return (0 != _id);
        }

        /**
         * @brief StepperMotorState::str
         * @return
         */
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

        int StepperMotorState::to_motor_pos(double pos_rad)
        {
            return static_cast<int32_t>((STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * _gear_ratio * pos_rad / (2*M_PI)) / _direction);
        }

        double StepperMotorState::to_rad_pos(int pos)
        {
            assert(0.0 != (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * _gear_ratio * RADIAN_TO_DEGREE));
            return static_cast<double>((pos * 2*M_PI) / (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * _gear_ratio) * _direction);
        }        


    } // namespace model
} // namespace common
