/*
    Math.hpp
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

#ifndef MATH_HPP
#define MATH_HPP

#define RADIAN_TO_DEGREE 57.295779513082320876798154814105

#define XL320_TOTAL_RANGE_POSITION 1023
#define XL320_MIDDLE_POSITION 511
#define XL320_TOTAL_ANGLE 296.67

#define XL430_TOTAL_RANGE_POSITION 4095
#define XL430_MIDDLE_POSITION 2047
#define XL430_TOTAL_ANGLE 360.36

#define STEPPERS_MICROSTEPS 8.0
#define STEPPERS_MOTOR_STEPS_PER_REVOLUTION 200.0

#include <stdint.h>

int32_t rad_pos_to_steps(double position_rad, double gear_ratio, double direction)
{
    return (int32_t)((STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * gear_ratio * position_rad * RADIAN_TO_DEGREE / 360.0) * direction);
};

double steps_to_rad_pos(int32_t steps, double gear_ratio, double direction)
{
    return (double)((double)steps * 360.0 / (STEPPERS_MOTOR_STEPS_PER_REVOLUTION * STEPPERS_MICROSTEPS * gear_ratio * RADIAN_TO_DEGREE)) * direction;
};

uint32_t rad_pos_to_xl320_pos(double position_rad)
{
    return (uint32_t)((double)XL320_MIDDLE_POSITION + (position_rad * RADIAN_TO_DEGREE * (double)XL320_TOTAL_RANGE_POSITION) / (double)XL320_TOTAL_ANGLE);
};

uint32_t rad_pos_to_xl430_pos(double position_rad)
{
    return (uint32_t)((double)XL430_MIDDLE_POSITION + (position_rad * RADIAN_TO_DEGREE * (double)XL430_TOTAL_RANGE_POSITION) / (double)XL430_TOTAL_ANGLE);
};

double xl320_pos_to_rad_pos(int32_t position_dxl)
{
    return (double)((((double)position_dxl - XL320_MIDDLE_POSITION) * (double)XL320_TOTAL_ANGLE) / (RADIAN_TO_DEGREE * (double)XL320_TOTAL_RANGE_POSITION));
};

double xl430_pos_to_rad_pos(int32_t position_dxl)
{
    return (double)((((double)position_dxl - XL430_MIDDLE_POSITION) * (double)XL430_TOTAL_ANGLE) / (RADIAN_TO_DEGREE * (double)XL430_TOTAL_RANGE_POSITION));
};
#endif