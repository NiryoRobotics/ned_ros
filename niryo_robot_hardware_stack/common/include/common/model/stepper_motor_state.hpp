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
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef STEPPER_MOTOR_STATE_H
#define STEPPER_MOTOR_STATE_H

#include <memory>
#include <string>
#include "joint_state.hpp"
#include "stepper_calibration_status_enum.hpp"

namespace common
{
    namespace model
    {

        struct VelocityProfile
        {
            uint32_t v_start{1};
            uint32_t a_1{0};
            uint32_t v_1{0};
            uint32_t a_max{6000};
            uint32_t v_max{6};
            uint32_t d_max{6000};
            uint32_t d_1{0};
            uint32_t v_stop{2};

            std::vector<uint32_t> to_list() const
            {
                return {v_start,
                        a_1,
                        v_1,
                        a_max,
                        v_max,
                        d_max,
                        d_1,
                        v_stop};
            }
        };

        /**
         * @brief The StepperMotorState class
         */
        class StepperMotorState : public JointState
        {

        public:
            StepperMotorState() = default;
            StepperMotorState(EHardwareType type, EComponentType component_type,
                              EBusProtocol bus_proto, uint8_t id);
            StepperMotorState(std::string name, EHardwareType type, EComponentType component_type,
                              EBusProtocol bus_proto, uint8_t id);

            int stepsPerRev();

            // setters
            void updateLastTimeRead();
            void setHwFailCounter(double fail_counter);
            void setMaxEffort(double max_effort);
            void setGearRatio(double gear_ratio);

            void setCalibration(const common::model::EStepperCalibrationStatus &calibration_state,
                                const int32_t &calibration_value);
            void setCalibration(const std::tuple<EStepperCalibrationStatus, int32_t> &data);
            void setMicroSteps(double micro_steps);
            void setMotorRatio(double motor_ratio);
            void setHomingAbsPosition(int32_t homing_abs_position);

            // getters
            double getLastTimeRead() const;
            double getHwFailCounter() const;

            double getMaxEffort() const;
            double getMicroSteps() const;
            int32_t getHomingAbsPosition() const;

            common::model::EStepperCalibrationStatus getCalibrationStatus() const;
            int32_t getCalibrationValue() const;

            VelocityProfile getVelocityProfile() const;
            void setVelocityProfile(const VelocityProfile &profile);

            // tests
            bool isConveyor() const;
            bool isTimeout() const;

            // JointState interface
            void reset() override;
            bool isValid() const override;
            std::string str() const override;

            int to_motor_pos(double rad_pos) override;
            double to_rad_pos(int motor_pos) override;

            int to_motor_vel(double rad_vel) override;
            double to_rad_vel(int motor_vel) override;

            void updateMultiplierRatio();

        protected:
            double _last_time_read{-1.0};
            double _hw_fail_counter{0.0};

            double _max_effort{0.0};
            double _micro_steps{8.0};
            double _gear_ratio{1.0};
            double _motor_ratio{1.0};  // ned2
            int32_t _homing_abs_position{0};  // ned2

            // profile
            VelocityProfile _profile;

            common::model::EStepperCalibrationStatus _calibration_status{common::model::EStepperCalibrationStatus::UNINITIALIZED};
            int32_t _calibration_value{0};

        private:
            double _pos_multiplier_ratio{1.0};
            double _vel_multiplier_ratio{1.0};

            static constexpr double STEPPERS_MOTOR_STEPS_PER_REVOLUTION = 200.0;
        };

        /**
         * @brief StepperMotorState::getMaxEffort
         * @return
         */
        inline double StepperMotorState::getMaxEffort() const
        {
            return _max_effort;
        }

        /**
         * @brief StepperMotorState::getCalibrationState
         * @return
         */
        inline EStepperCalibrationStatus StepperMotorState::getCalibrationStatus() const
        {
            return _calibration_status;
        }

        /**
         * @brief StepperMotorState::getCalibrationValue
         * @return
         */
        inline int32_t StepperMotorState::getCalibrationValue() const
        {
            return _calibration_value;
        }

        /**
         * @brief StepperMotorState::isConveyor
         * @return
         */
        inline bool StepperMotorState::isConveyor() const
        {
            return (getComponentType() == common::model::EComponentType::CONVEYOR);
        }

        /**
         * @brief StepperMotorState::getVelocityProfile
         * @return
         */
        inline VelocityProfile StepperMotorState::getVelocityProfile() const
        {
            return _profile;
        }

        /**
         * @brief StepperMotorState::getMicroSteps
         * @return
         */
        inline double StepperMotorState::getMicroSteps() const
        {
            return _micro_steps;
        }

        /**
         * @brief StepperMotorState::getMicroSteps
         * @return
         */
        inline int32_t StepperMotorState::getHomingAbsPosition() const
        {
            return _homing_abs_position;
        }

        /**
         * @brief StepperMotorState::getLastTimeRead
         * @return
         */
        inline int StepperMotorState::stepsPerRev()
        {
            return int(_micro_steps * STEPPERS_MOTOR_STEPS_PER_REVOLUTION);
        }

        inline double StepperMotorState::getLastTimeRead() const
        {
            return _last_time_read;
        }

        /**
         * @brief StepperMotorState::getHwFailCounter
         * @return
         */
        inline double StepperMotorState::getHwFailCounter() const
        {
            return _hw_fail_counter;
        }

    } // model
} // common

#endif
