/*
stepper_calibration_status_enum.hpp
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

#ifndef STEPPER_CALIBRATION_STATUS_ENUM_H
#define STEPPER_CALIBRATION_STATUS_ENUM_H

#include <map>
#include <string>

#include "common/common_defs.hpp"
#include "abstract_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The EStepperCalibrationStatus enum
 */
enum class EStepperCalibrationStatus {
    UNINITIALIZED = 0, // means a calibration is expected
    OK = 1,
    TIMEOUT = 2,
    BAD_PARAM = 3,
    FAIL = 4,
    WAITING_USER_INPUT = 5,
    IN_PROGRESS = 6,
};

/**
 * @brief Specialization of AbstractEnum for Acknowledge status enum
 */
class StepperCalibrationStatusEnum : public AbstractEnum<StepperCalibrationStatusEnum, EStepperCalibrationStatus>
{
public:
    StepperCalibrationStatusEnum(EStepperCalibrationStatus e=EStepperCalibrationStatus::UNINITIALIZED);
    StepperCalibrationStatusEnum(const char* str);

private:
    friend class AbstractEnum<StepperCalibrationStatusEnum, EStepperCalibrationStatus>;
    static std::map<EStepperCalibrationStatus, std::string> initialize();
};

} // model
} // common

#endif // STEPPER_CALIBRATION_STATUS_ENUM_H
