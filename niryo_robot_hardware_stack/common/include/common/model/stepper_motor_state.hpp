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
#include "stepper_calibration_status_enum.hpp"

namespace common {
    namespace model {

        /**
         * @brief The StepperMotorState class
         */
        class StepperMotorState : public JointState
        {

            public:
                StepperMotorState();
                StepperMotorState(uint8_t id, bool isConveyor = false);
                StepperMotorState(std::string name, EMotorType type, uint8_t id, bool isConveyor = false );

                virtual ~StepperMotorState() override;

                static constexpr int stepsPerRev();

                //setters
                void updateLastTimeRead();
                void setHwFailCounter(double fail_counter);
                void setFirmwareVersion(std::string& firmware_version);
                void setGearRatio(double gear_ratio);
                void setDirection(double direction);
                void setMaxEffort(double max_effort);

                void setCalibration(const common::model::EStepperCalibrationStatus &calibration_state,
                                    const int32_t &calibration_value);

                //getters
                double getLastTimeRead() const;
                double getHwFailCounter() const;
                std::string getFirmwareVersion() const;

                double getGearRatio() const;
                double getDirection() const;
                double getMaxEffort() const;

                common::model::EStepperCalibrationStatus getCalibrationState() const;
                int32_t getCalibrationValue() const;

                //tests
                bool isConveyor() const;
                bool isTimeout() const;

                // JointState interface
                virtual void reset() override;
                virtual bool isValid() const override;
                virtual std::string str() const override;

                virtual int to_motor_pos(double pos_rad) override;
                virtual double to_rad_pos(int pos) override;

        protected:
                bool _isConveyor{false};

                double _last_time_read{-1.0};
                double _hw_fail_counter{0.0};

                double _gear_ratio{0.0};
                double _direction{0.0};
                double _max_effort{0.0};

                std::string _firmware_version;

                common::model::EStepperCalibrationStatus _calibration_state{common::model::EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED};
                int32_t _calibration_value{0};

            private:

                static constexpr double STEPPERS_MICROSTEPS                 = 8.0;
                static constexpr double STEPPERS_MOTOR_STEPS_PER_REVOLUTION = 200.0;

        };

        /**
         * @brief StepperMotorState::getMaxEffort
         * @return
         */
        inline
        double StepperMotorState::getMaxEffort() const
        {
            return _max_effort;
        }

        /**
         * @brief StepperMotorState::getCalibrationState
         * @return
         */
        inline
        EStepperCalibrationStatus StepperMotorState::getCalibrationState() const
        {
            return _calibration_state;
        }

        /**
         * @brief StepperMotorState::getCalibrationValue
         * @return
         */
        inline
        int32_t StepperMotorState::getCalibrationValue() const
        {
            return _calibration_value;
        }

        /**
         * @brief StepperMotorState::isConveyor
         * @return
         */
        inline
        bool StepperMotorState::isConveyor() const
        {
            return _isConveyor;
        }

        /**
         * @brief StepperMotorState::getGearRatio
         * @return
         */
        inline
        double StepperMotorState::getGearRatio() const
        {
            return _gear_ratio;
        }

        /**
         * @brief StepperMotorState::getDirection
         * @return
         */
        inline
        double StepperMotorState::getDirection() const
        {
            return _direction;
        }

        /**
         * @brief StepperMotorState::getLastTimeRead
         * @return
         */
        constexpr int StepperMotorState::stepsPerRev() {
            return int(STEPPERS_MICROSTEPS * STEPPERS_MOTOR_STEPS_PER_REVOLUTION);
        }

        inline
        double StepperMotorState::getLastTimeRead() const
        {
            return _last_time_read;
        }

        /**
         * @brief StepperMotorState::getHwFailCounter
         * @return
         */
        inline
        double StepperMotorState::getHwFailCounter() const
        {
            return _hw_fail_counter;
        }

        /**
         * @brief StepperMotorState::getFirmwareVersion
         * @return
         */
        inline
        std::string StepperMotorState::getFirmwareVersion() const
        {
            return _firmware_version;
        }

    } // model
} // common

#endif
