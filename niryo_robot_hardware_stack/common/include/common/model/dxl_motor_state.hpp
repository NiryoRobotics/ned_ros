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
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef DXL_MOTOR_STATE_H
#define DXL_MOTOR_STATE_H

#include <string>
#include "joint_state.hpp"

namespace common
{
namespace model
{

/**
 * @brief The DxlMotorState class
 */
class DxlMotorState : public JointState
{
    public:
        DxlMotorState();
        DxlMotorState(uint8_t id);
        DxlMotorState(EHardwareType type, EComponentType component_type,
                      uint8_t id);

        DxlMotorState(std::string name, EHardwareType type, EComponentType component_type,
                      uint8_t id);

        // getters
        bool isTool() const;

        // JointState interface
        std::string str() const override;
        void reset() override;
        bool isValid() const override;

        int to_motor_pos(double rad_pos) override;
        double to_rad_pos(int motor_pos) override;

        int to_motor_vel(double rad_vel) override;
        double to_rad_vel(int motor_vel) override;

        uint32_t getPositionPGain() const;
        uint32_t getPositionIGain() const;
        uint32_t getPositionDGain() const;

        uint32_t getVelocityPGain() const;
        uint32_t getVelocityIGain() const;

        uint32_t getFF1Gain() const;
        uint32_t getFF2Gain() const;

        uint32_t getVelProfile() const;
        uint32_t getAccProfile() const;

        double getStepsForOneSpeed() const;
        int getTotalRangePosition() const;
        double getTotalAngle() const;

        void setPositionPGain(uint32_t p_gain);
        void setPositionIGain(uint32_t i_gain);
        void setPositionDGain(uint32_t d_gain);

        void setVelocityPGain(uint32_t p_gain);
        void setVelocityIGain(uint32_t i_gain);

        void setFF1Gain(uint32_t ff1_gain);
        void setFF2Gain(uint32_t ff2_gain);

        void setVelProfile(uint32_t vel_profile);
        void setAccProfile(uint32_t acc_profile);

protected:
        uint32_t _pos_p_gain{0};
        uint32_t _pos_i_gain{0};
        uint32_t _pos_d_gain{0};

        uint32_t _vel_p_gain{0};
        uint32_t _vel_i_gain{0};

        uint32_t _ff1_gain{0};
        uint32_t _ff2_gain{0};
        uint32_t _vel_profile{0};
        uint32_t _acc_profile{0};        

        int _total_range_position{0};
        double _total_angle{0.0};
        double _steps_for_one_speed{0.0};

private:
        void updateMultiplierRatio();

        double _pos_multiplier_ratio{1.0};
        double _vel_multiplier_ratio{0.229};
};

/**
 * @brief DxlMotorState::isTool
 * @return
 */
inline
bool DxlMotorState::isTool() const
{
    return (getComponentType() == common::model::EComponentType::TOOL);
}

/**
 * @brief DxlMotorState::getPositionPGain
 * @return
 */
inline
uint32_t DxlMotorState::getPositionPGain() const
{
    return _pos_p_gain;
}

/**
 * @brief DxlMotorState::getPositionIGain
 * @return
 */
inline
uint32_t DxlMotorState::getPositionIGain() const
{
    return _pos_i_gain;
}

/**
 * @brief DxlMotorState::getPositionDGain
 * @return
 */

inline
uint32_t DxlMotorState::getPositionDGain() const
{
    return _pos_d_gain;
}

/**
 * @brief DxlMotorState::getVelocityPGain
 * @return
 */
inline
uint32_t DxlMotorState::getVelocityPGain() const
{
    return _vel_p_gain;
}

/**
 * @brief DxlMotorState::getVelocityIGain
 * @return
 */
inline
uint32_t DxlMotorState::getVelocityIGain() const
{
    return _vel_i_gain;
}

/**
 * @brief DxlMotorState::getFF1Gain
 * @return
 */
inline
uint32_t DxlMotorState::getFF1Gain() const
{
    return _ff1_gain;
}

/**
 * @brief DxlMotorState::getFF2Gain
 * @return
 */
inline
uint32_t DxlMotorState::getFF2Gain() const
{
    return _ff2_gain;
}

/**
 * @brief DxlMotorState::getVelProfile
 * @return
 */
inline
uint32_t DxlMotorState::getVelProfile() const
{
    return _vel_profile;
}

/**
 * @brief DxlMotorState::getAccProfile
 * @return
 */
inline
uint32_t DxlMotorState::getAccProfile() const
{
    return _acc_profile;
}

/**
 * @brief DxlMotorState::getStepsForOneSpeed
 * @return
 */
inline
double DxlMotorState::getStepsForOneSpeed() const
{
  return _steps_for_one_speed;
}

/**
 * @brief DxlMotorState::getTotalRangePosition
 * @return
 */
inline
int DxlMotorState::getTotalRangePosition() const
{
  return _total_range_position;
}

/**
 * @brief DxlMotorState::getTotalAngle
 * @return
 */
inline
double DxlMotorState::getTotalAngle() const
{
  return _total_angle;
}

} // namespace model
} // namespace common

#endif
