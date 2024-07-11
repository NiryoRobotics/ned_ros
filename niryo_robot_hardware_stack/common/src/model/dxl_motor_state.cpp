/*
    dxl_motor_state.cpp
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

#include "common/model/dxl_motor_state.hpp"

#include <cassert>
#include <cmath>
#include <sstream>
#include <string>
#include <utility>

namespace common
{
namespace model
{

/**
 * @brief DxlMotorState::DxlMotorState
 */
DxlMotorState::DxlMotorState() { reset(); }

/**
 * @brief DxlMotorState::DxlMotorState
 */
DxlMotorState::DxlMotorState(uint8_t id) : DxlMotorState("unknown", EHardwareType::UNKNOWN, EComponentType::JOINT, id) {}

/**
 * @brief DxlMotorState::DxlMotorState
 * @param type
 * @param component_type
 * @param id
 */
DxlMotorState::DxlMotorState(EHardwareType type, EComponentType component_type, uint8_t id) : DxlMotorState("unknown", type, component_type, id) {}

/**
 * @brief DxlMotorState::DxlMotorState
 * @param name
 * @param type
 * @param component_type
 * @param id
 */
DxlMotorState::DxlMotorState(std::string name, EHardwareType type, EComponentType component_type, uint8_t id)
    : JointState(std::move(name), type, component_type, EBusProtocol::TTL, id)
{
    // to put in config ?

    // according to xl-320 datasheet : 1 speed ~ 0.111 rpm ~ 1.8944 dxl position per second
    switch (_hw_type)
    {
    case EHardwareType::XL320:
        _total_angle = 300;
        _total_range_position = 1024;
        _steps_for_one_speed = 1.8944;  // 0.111 * 1024 / 60
        break;
    case EHardwareType::XC430:
        _total_angle = 360;
        _total_range_position = 4096;
        _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        break;
    case EHardwareType::XL330:
        _total_angle = 360;
        _total_range_position = 4096;
        _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        break;
    case EHardwareType::XL430:
        _total_angle = 360;
        _total_range_position = 4096;
        _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        break;
    case EHardwareType::XM430:
        _total_angle = 360;
        _total_range_position = 4096;
        _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        break;
    case EHardwareType::XH430:
        _total_angle = 360;
        _total_range_position = 4096;
        _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        break;
    case EHardwareType::FAKE_DXL_MOTOR:
        _total_angle = 360;
        _total_range_position = 4096;
        _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        break;
    default:
        break;
    }

    updateMultiplierRatio();
}

// *********************
//  JointState Interface
// ********************

/**
 * @brief DxlMotorState::reset
 */
void DxlMotorState::reset() { common::model::JointState::reset(); }

/**
 * @brief DxlMotorState::isValid
 * @return
 */
bool DxlMotorState::isValid() const { return (0 != getId() && EHardwareType::UNKNOWN != getHardwareType()); }

/**
 * @brief DxlMotorState::str
 * @return
 */
std::string DxlMotorState::str() const
{
    std::ostringstream ss;

    ss << "DxlMotorState : ";
    ss << "isTool: " << (isTool() ? "true" : "false") << "\n";

    ss << "position p gain: " << _pos_p_gain << ", "
       << "position i gain: " << _pos_i_gain << ", "
       << "position d gain: " << _pos_d_gain << ",\n"
       << "velocity p gain: " << _vel_p_gain << ", "
       << "velocity i gain: " << _vel_i_gain << ",\n"
       << "ff1 gain: " << _ff1_gain << ", "
       << "ff2 gain: " << _ff2_gain << ",\n"
       << "velocity profile: " << _vel_profile << ", "
       << "acceleration profile: " << _acc_profile << ",\n";

    ss << "total range position: " << _total_range_position << ", "
       << "total angle: " << _total_angle << ", "
       << "pos multiplier ratio: " << _pos_multiplier_ratio << ", "
       << "vel multiplier ratio: " << _vel_multiplier_ratio << ", "
       << "steps for one speed: " << _steps_for_one_speed;

    ss << "\n---\n";
    ss << "\n";
    ss << JointState::str();

    return ss.str();
}

/**
 * @brief DxlMotorState::to_motor_pos
 * @param rad_pos
 * @return
 */
int DxlMotorState::to_motor_pos(double rad_pos)
{
    if (rad_pos > _limit_position_max)
        rad_pos = _limit_position_max;
    else if (rad_pos < _limit_position_min)
        rad_pos = _limit_position_min;
    return static_cast<int>(std::round((rad_pos - _offset_position) * _pos_multiplier_ratio * _direction));
}

/**
 * @brief DxlMotorState::to_rad_pos
 * @param motor_pos
 * @return
 */
double DxlMotorState::to_rad_pos(int motor_pos)
{
    assert(0.0 != _pos_multiplier_ratio * _direction);

    return _offset_position + (motor_pos / (_pos_multiplier_ratio * _direction));
}

/**
 * @brief DxlMotorState::to_motor_vel
 * @param rad_vel
 * @return
 */
int DxlMotorState::to_motor_vel(double rad_vel) { return static_cast<int>(std::round(rad_vel / _vel_multiplier_ratio)); }

/**
 * @brief DxlMotorState::to_motor_vel
 * @param motor_vel
 * @return
 */
double DxlMotorState::to_rad_vel(int motor_vel) { return motor_vel * _vel_multiplier_ratio; }

/**
 * @brief DxlMotorState::setPositionPGain
 * @param p_gain
 */
void DxlMotorState::setPositionPGain(uint32_t p_gain) { _pos_p_gain = p_gain; }

/**
 * @brief DxlMotorState::setPositionIGain
 * @param i_gain
 */
void DxlMotorState::setPositionIGain(uint32_t i_gain) { _pos_i_gain = i_gain; }

/**
 * @brief DxlMotorState::setPositionDGain
 * @param d_gain
 */
void DxlMotorState::setPositionDGain(uint32_t d_gain) { _pos_d_gain = d_gain; }

/**
 * @brief DxlMotorState::setVelocityPGain
 * @param p_gain
 */
void DxlMotorState::setVelocityPGain(uint32_t p_gain) { _vel_p_gain = p_gain; }

/**
 * @brief DxlMotorState::setPositionIGain
 * @param i_gain
 */
void DxlMotorState::setVelocityIGain(uint32_t i_gain) { _vel_i_gain = i_gain; }

/**
 * @brief DxlMotorState::setFF1Gain
 * @param ff1_gain
 */
void DxlMotorState::setFF1Gain(uint32_t ff1_gain) { _ff1_gain = ff1_gain; }

/**
 * @brief DxlMotorState::setFF2Gain
 * @param ff2_gain
 */
void DxlMotorState::setFF2Gain(uint32_t ff2_gain) { _ff2_gain = ff2_gain; }

/**
 * @brief DxlMotorState::setVelProfile
 * @param vel_profile
 */
void DxlMotorState::setVelProfile(uint32_t vel_profile) { _vel_profile = vel_profile; }

/**
 * @brief DxlMotorState::setAccProfile
 * @param acc_profile
 */
void DxlMotorState::setAccProfile(uint32_t acc_profile) { _acc_profile = acc_profile; }

/**
 * @brief DxlMotorState::updateMultiplierRatio
 */
void DxlMotorState::updateMultiplierRatio()
{
    assert(0.0 != _total_angle);

    _pos_multiplier_ratio = RADIAN_TO_DEGREE * _total_range_position / _total_angle;
    _vel_multiplier_ratio = 0.229;
}

}  // namespace model
}  // namespace common
