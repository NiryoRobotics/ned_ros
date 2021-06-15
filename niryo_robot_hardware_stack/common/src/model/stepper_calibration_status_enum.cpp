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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "common/model/stepper_calibration_status_enum.hpp"

using namespace std;

namespace common {
    namespace model {

        /**
         * @brief StepperCalibrationStatusEnum::StepperCalibrationStatusEnum
         * @param e
         */
        StepperCalibrationStatusEnum::StepperCalibrationStatusEnum(EStepperCalibrationStatus e):
            AbstractEnum<StepperCalibrationStatusEnum, EStepperCalibrationStatus>(e)
        {}

        /**
         * @brief StepperCalibrationStatusEnum::StepperCalibrationStatusEnum
         * @param str
         */
        StepperCalibrationStatusEnum::StepperCalibrationStatusEnum(const char* const str):
            AbstractEnum<StepperCalibrationStatusEnum, EStepperCalibrationStatus>(str)
        {}

        /**
         * @brief StepperCalibrationStatusEnum::initialize
         * @return
         */
        map<EStepperCalibrationStatus, string>
        StepperCalibrationStatusEnum::initialize()
        {
            map<EStepperCalibrationStatus, string> m;

            m[EStepperCalibrationStatus::CALIBRATION_UNINITIALIZED] = "uninitialized";
            m[EStepperCalibrationStatus::CALIBRATION_OK]    = "ok";
            m[EStepperCalibrationStatus::CALIBRATION_TIMEOUT]     = "timeout";
            m[EStepperCalibrationStatus::CALIBRATION_BAD_PARAM]    = "bad parameter";
            m[EStepperCalibrationStatus::CALIBRATION_FAIL] = "fail";
            m[EStepperCalibrationStatus::CALIBRATION_WAITING_USER_INPUT]  = "waiting user input";
            m[EStepperCalibrationStatus::CALIBRATION_IN_PROGRESS]  = "in progress";

            return m;
        }

    } // model
} //common

