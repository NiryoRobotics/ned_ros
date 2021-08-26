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

#include <sstream>
#include <math.h>
#include <cassert>
#include <cmath>
#include <string>

namespace common
{
namespace model
{

/**
 * @brief DxlMotorState::DxlMotorState
 */
DxlMotorState::DxlMotorState()
    : JointState()
{
    reset();
}

/**
 * @brief DxlMotorState::DxlMotorState
 * @param name
 * @param type
 * @param bus_proto
 * @param id
 * @param isTool
 */
DxlMotorState::DxlMotorState(std::string name,
                             EMotorType type,
                             EBusProtocol bus_proto,
                             uint8_t id,
                             bool isTool) :
    JointState(name, type, bus_proto, id),
    _isTool(isTool)
{
    // to put in config ?

    // according to xl-320 datasheet : 1 speed ~ 0.111 rpm ~ 1.8944 dxl position per second
    switch (_type)
    {
        case EMotorType::XL320:
            _total_angle = 300;
            _total_range_position = 1024;
            _middle_position = 512;
            _steps_for_one_speed =  1.8944;  // 0.111 * 1024 / 60
        break;
        case EMotorType::XC430:
            _total_angle = 360;
            _total_range_position = 4096;
            _middle_position = 2048;
            _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        break;
        case EMotorType::XL330:
            _total_angle = 360;
            _total_range_position = 4096;
            _middle_position = 2048;
            _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        break;
        case EMotorType::XL430:
            _total_angle = 360;
            _total_range_position = 4096;
            _middle_position = 2048;
            _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        case EMotorType::FAKE_DXL_MOTOR:
            _total_angle = 360;
            _total_range_position = 4096;
            _middle_position = 2048;
            _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        case EMotorType::FAKE_STEPPER_MOTOR:
            _total_angle = 360;
            _total_range_position = 4096;
            _middle_position = 2048;
            _steps_for_one_speed = 15.6330667;  // 0.229 * 4096 / 60
        break;
        default:
        break;
    }
}

/**
 * @brief DxlMotorState::DxlMotorState
 * @param type
 * @param bus_proto
 * @param id
 * @param isTool
 */
DxlMotorState::DxlMotorState(EMotorType type,
                             EBusProtocol bus_proto,
                             uint8_t id,
                             bool isTool) :
    DxlMotorState("unknown", type, bus_proto, id, isTool)
{}

/**
 * @brief DxlMotorState::~DxlMotorState
 */
DxlMotorState::~DxlMotorState()
{}

// *********************
//  JointState Interface
// ********************

/**
 * @brief DxlMotorState::reset
 */
void DxlMotorState::reset()
{
    AbstractMotorState::reset();
    _isTool = false;
}

/**
 * @brief DxlMotorState::isValid
 * @return
 */
bool DxlMotorState::isValid() const
{
    return (0 != getId() && EMotorType::UNKNOWN != getType());
}

/**
 * @brief DxlMotorState::str
 * @return
 */
std::string DxlMotorState::str() const
{
    std::ostringstream ss;

    ss << "DxlMotorState : ";
    ss << "\n---\n";
    ss << "type: " << MotorTypeEnum(_type).toString() << ", ";
    ss << "isTool: " << (_isTool ? "true" : "false") << "\n";
    ss << "position p gain: " << _pos_p_gain << ", ";
    ss << "position i gain: " << _pos_i_gain << ", ";
    ss << "position d gain: " << _pos_d_gain << ", ";
    ss << "velocity p gain: " << _vel_p_gain << ", ";
    ss << "velocity i gain: " << _vel_i_gain << ", ";
    ss << "ff1 gain: " << _ff1_gain << ", ";
    ss << "ff2 gain: " << _ff2_gain << ", ";
    ss << "\n";
    ss << JointState::str();

    return ss.str();
}

/**
 * @brief DxlMotorState::to_motor_pos
 * @param pos_rad
 * @return
 */
int DxlMotorState::to_motor_pos(double pos_rad)
{
    double denominator = _total_angle;
    assert(0.0 != denominator);
    double numerator = ((pos_rad - _offset_position) * RADIAN_TO_DEGREE * _total_range_position);

    return _middle_position + std::round(numerator / denominator) * _direction;
}

/**
 * @brief DxlMotorState::to_rad_pos
 * @param position_dxl
 * @return
 */
double DxlMotorState::to_rad_pos(int position_dxl)
{
    double denominator = RADIAN_TO_DEGREE * _total_range_position;
    assert(0.0 != denominator);
    double numerator = (position_dxl - _middle_position) * _total_angle;

    return _offset_position + (numerator / denominator) * _direction;
}

/**
 * @brief DxlMotorState::getTotalRangePosition
 * @return
 */
int DxlMotorState::getTotalRangePosition() const
{
    return _total_range_position;
}

/**
 * @brief DxlMotorState::getMiddlePosition
 * @return
 */
int DxlMotorState::getMiddlePosition() const
{
    return _middle_position;
}

/**
 * @brief DxlMotorState::getTotalAngle
 * @return
 */
double DxlMotorState::getTotalAngle() const
{
    return _total_angle;
}

/**
 * @brief DxlMotorState::getStepsForOneSpeed
 * @return
 */
double DxlMotorState::getStepsForOneSpeed() const
{
    return _steps_for_one_speed;
}

/**
 * @brief DxlMotorState::setPositionPGain
 * @param p_gain
 */
void DxlMotorState::setPositionPGain(uint32_t p_gain)
{
    _pos_p_gain = p_gain;
}

/**
 * @brief DxlMotorState::setPositionIGain
 * @param i_gain
 */
void DxlMotorState::setPositionIGain(uint32_t i_gain)
{
    _pos_i_gain = i_gain;
}

/**
 * @brief DxlMotorState::setPositionDGain
 * @param d_gain
 */
void DxlMotorState::setPositionDGain(uint32_t d_gain)
{
    _pos_d_gain = d_gain;
}

/**
 * @brief DxlMotorState::setVelocityPGain
 * @param p_gain
 */
void DxlMotorState::setVelocityPGain(uint32_t p_gain)
{
    _vel_p_gain = p_gain;
}

/**
 * @brief DxlMotorState::setPositionIGain
 * @param i_gain
 */
void DxlMotorState::setVelocityIGain(uint32_t i_gain)
{
    _vel_i_gain = i_gain;
}

/**
 * @brief DxlMotorState::setFF1Gain
 * @param ff1_gain
 */
void DxlMotorState::setFF1Gain(uint32_t ff1_gain)
{
    _ff1_gain = ff1_gain;
}

/**
 * @brief DxlMotorState::setFF2Gain
 * @param value
 */
void DxlMotorState::setFF2Gain(uint32_t value)
{
    _ff2_gain = value;
}

}  // namespace model
}  // namespace common
