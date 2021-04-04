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

#include "utils/motor_state.hpp"
#include <stdint.h>
#include <string>


namespace JointsInterface {

    class JointState : public utils::MotorState
    {

        public:
            JointState();
            JointState(std::string name, uint8_t type, uint8_t id );

            void reset();
            bool isValid() const;

            std::string getName() const;
            void setName(std::string &name);

            uint8_t getType() const;
            void setType(uint8_t id);

            bool needCalibration() const;
            void setNeedCalibration(bool need_calibration);

            void setPosition(double position);
            double getPosition() const;

            bool operator==(const JointState& other) const;

        protected :

            std::string _name;
            uint8_t _type;

            double _position;
            bool _need_calibration;

    };

    inline
    bool JointState::isValid() const
    {
        return (0 != _id && 0 != _type);
    }

    inline
    std::string JointState::getName() const


    {
        return _name;
    }

    inline
    uint8_t JointState::getType() const
    {
        return _type;
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

} //JointsInterface

#endif
