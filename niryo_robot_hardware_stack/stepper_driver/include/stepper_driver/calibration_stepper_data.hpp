
/*
    calibration_state.hpp
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

#ifndef CALIBRATION_STEPPER_DATA_HPP
#define CALIBRATION_STEPPER_DATA_HPP

#include <string>
#include <vector>

#include <ros/time.h>

#include "model/iobject.hpp"
#include "model/stepper_command_type_enum.hpp"
#include "model/stepper_motor_cmd.hpp"
#include "model/stepper_calibration_status_enum.hpp"

namespace StepperDriver
{
    class CalibrationStepperData : public common::model::IObject
    {
        public:
            CalibrationStepperData();
            virtual ~CalibrationStepperData() override;

            CalibrationStepperData(unsigned long irxId, uint8_t ilen, std::array<uint8_t, 8> irxBuf);

            uint8_t getId() const;

            common::model::EStepperCalibrationStatus getStatus() const;

            int32_t getCalibrationResult() const;

            // IObject interface
        public:
            virtual void reset() override;
            virtual std::string str() const override;
            virtual bool isValid() const override;

        private:
            static constexpr int CAN_DATA_CALIBRATION_RESULT = 0x09;

            unsigned long _rxId;
            uint8_t _len;
            std::array<uint8_t, 8> _rxBuf;

            ros::Time _time;
    };

} // namespace StepperDriver

#endif
