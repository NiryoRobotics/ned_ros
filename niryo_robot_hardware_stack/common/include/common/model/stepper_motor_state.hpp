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

/**
 * @brief The StepperMotorState class
 */
class StepperMotorState : public JointState
{

    public:
        StepperMotorState();
        StepperMotorState(EHardwareType type, EComponentType component_type,
                          EBusProtocol bus_proto, uint8_t id);
        StepperMotorState(std::string name, EHardwareType type, EComponentType component_type,
                          EBusProtocol bus_proto, uint8_t id);
        StepperMotorState(const StepperMotorState& state);

        virtual ~StepperMotorState() override;

        int stepsPerRev();

        // setters
        void updateLastTimeRead();
        void setHwFailCounter(double fail_counter);
        void setGearRatio(double gear_ratio);
        void setMaxEffort(double max_effort);

        void setCalibration(const common::model::EStepperCalibrationStatus &calibration_state,
                            const int32_t &calibration_value);
        void setCalibration(const std::tuple<EStepperCalibrationStatus, int32_t> &data);

        // getters
        double getLastTimeRead() const;
        double getHwFailCounter() const;

        double getGearRatio() const;
        double getMaxEffort() const;

        common::model::EStepperCalibrationStatus getCalibrationState() const;
        int32_t getCalibrationValue() const;

        // tests
        bool isConveyor() const;
        bool isTimeout() const;

        // JointState interface
        virtual void reset() override;
        virtual bool isValid() const override;
        virtual std::string str() const override;

        virtual int to_motor_pos(double pos_rad) override;
        virtual double to_rad_pos(int pos) override;

        std::vector<uint32_t> getVelocityProfile() const;

        double getMicroSteps() const;
        void setMicroSteps(double micro_steps);

        uint32_t getProfileVStart() const;
        void setProfileVStart(const uint32_t &profile_v_start);

        uint32_t getProfileA1() const;
        void setProfileA1(const uint32_t &profile_a_1);

        uint32_t getProfileV1() const;
        void setProfileV1(const uint32_t &profile_v_1);

        uint32_t getProfileAMax() const;
        void setProfileAMax(const uint32_t &profile_a_max);

        uint32_t getProfileVMax() const;
        void setProfileVMax(const uint32_t &profile_v_max);

        uint32_t getProfileDMax() const;
        void setProfileDMax(const uint32_t &profile_d_max);

        uint32_t getProfileD1() const;
        void setProfileD1(const uint32_t &profile_d_1);

        uint32_t getProfileVStop() const;
        void setProfileVStop(const uint32_t &profile_v_stop);

protected:
        double _last_time_read{-1.0};
        double _hw_fail_counter{0.0};

        double _gear_ratio{1.0};
        double _max_effort{0.0};
        double _micro_steps{8.0};

        // profile
        uint32_t _profile_v_start{1};
        uint32_t _profile_a_1{0};
        uint32_t _profile_v_1{0};
        uint32_t _profile_a_max{6000};
        uint32_t _profile_v_max{6};
        uint32_t _profile_d_max{6000};
        uint32_t _profile_d_1{0};
        uint32_t _profile_v_stop{2};

        common::model::EStepperCalibrationStatus _calibration_state{common::model::EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED};
        int32_t _calibration_value{0};

    private:
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
    return (getComponentType() == common::model::EComponentType::CONVEYOR);
}

/**
 * @brief StepperMotorState::getVelocityProfile
 * @return
 */
inline
std::vector<uint32_t> StepperMotorState::getVelocityProfile() const
{
  return {_profile_v_start,
          _profile_a_1,
          _profile_v_1,
          _profile_a_max,
          _profile_v_max,
          _profile_d_max,
          _profile_d_1,
          _profile_v_stop};
}

/**
 * @brief StepperMotorState::getMicroSteps
 * @return
 */
inline
double StepperMotorState::getMicroSteps() const
{
  return _micro_steps;
}

/**
 * @brief StepperMotorState::getProfileVStart
 * @return
 */
inline
uint32_t StepperMotorState::getProfileVStart() const
{
  return _profile_v_start;
}

/**
 * @brief StepperMotorState::getProfileA1
 * @return
 */
inline
uint32_t StepperMotorState::getProfileA1() const
{
  return _profile_a_1;
}

/**
 * @brief StepperMotorState::getProfileV1
 * @return
 */
inline
uint32_t StepperMotorState::getProfileV1() const
{
  return _profile_v_1;
}

/**
 * @brief StepperMotorState::getProfileAMax
 * @return
 */
inline
uint32_t StepperMotorState::getProfileAMax() const
{
  return _profile_a_max;
}

/**
 * @brief StepperMotorState::getProfileVMax
 * @return
 */
inline
uint32_t StepperMotorState::getProfileVMax() const
{
  return _profile_v_max;
}

/**
 * @brief StepperMotorState::getProfileDMax
 * @return
 */
inline
uint32_t StepperMotorState::getProfileDMax() const
{
  return _profile_d_max;
}

/**
 * @brief StepperMotorState::getProfileD1
 * @return
 */
inline
uint32_t StepperMotorState::getProfileD1() const
{
  return _profile_d_1;
}

/**
 * @brief StepperMotorState::getProfileVStop
 * @return
 */
inline
uint32_t StepperMotorState::getProfileVStop() const
{
  return _profile_v_stop;
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
 * @brief StepperMotorState::getLastTimeRead
 * @return
 */
inline
int StepperMotorState::stepsPerRev() {
    return int(_micro_steps * STEPPERS_MOTOR_STEPS_PER_REVOLUTION);
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


} // model
} // common

#endif
