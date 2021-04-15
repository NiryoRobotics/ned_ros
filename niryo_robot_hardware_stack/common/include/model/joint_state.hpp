/*
    joint_state.hpp
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

#ifndef JOINT_STATE_HPP
#define JOINT_STATE_HPP

#include "abstract_motor_state.hpp"
#include "motor_type_enum.hpp"

#include <stdint.h>
#include <string>


namespace common {
    namespace model {

        class JointState : public AbstractMotorState
        {

            public:
                JointState();
                JointState(std::string name, EMotorType type, uint8_t id );

                virtual ~JointState() override;

                void setName(std::string &name);
                void setNeedCalibration(bool need_calibration);
                void setPosition(double position);

                std::string getName() const;
                bool needCalibration() const;
                double getPosition() const;

                bool operator==(const JointState& other) const;

                // AbstractMotorState interface
                virtual void reset() override;
                virtual bool isValid() const override;
                virtual std::string str() const override;

            protected:

                std::string _name;
                double _position;
                bool _need_calibration;
        };

        inline
        bool JointState::isValid() const
        {
            return (0 != getId() && EMotorType::MOTOR_TYPE_UNKNOWN != getType());
        }

        inline
        std::string JointState::getName() const
        {
            return _name;
        }

        inline
        bool JointState::needCalibration() const
        {
            return _need_calibration;
        }

        inline
        double JointState::getPosition() const
        {
            return _position;
        }

    } // namespace JointsInterface
} // namespace common
#endif
