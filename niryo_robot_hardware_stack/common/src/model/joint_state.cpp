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

#include "model/joint_state.hpp"
#include <sstream>

using namespace std;

namespace common {
    namespace model {

        JointState::JointState() :
            AbstractMotorState()
        {
        }

        JointState::JointState(string name, EMotorType type, uint8_t id) :
            AbstractMotorState(id, type),
            _name(name)
        {
        }

        JointState::~JointState()
        {

        }


        bool JointState::operator==(const JointState& m) const
        {
            return((this->_type == m._type) && (this->_id == m._id));
        }


        void JointState::setName(string& name)
        {
            _name = name;
        }

        void JointState::setNeedCalibration(bool need_calibration)
        {
            _need_calibration = need_calibration;
        }

        void JointState::setPosition(double position)
        {
            _position = position;
        }


        void JointState::reset()
        {
            AbstractMotorState::reset();
            _name.clear();
            _position = 0.0;
            _need_calibration = false;
        }

        string JointState::str() const
        {
            ostringstream ss;

            ss << "JointState : ";
            ss << "\n---\n";
            ss << "type: " << MotorTypeEnum(_type).toString() << ", ";
            ss << "name: " << "\"" << _name << "\"" << ", ";
            ss << "position: " << _position << ", ";
            ss << "need calibration: " << (_need_calibration ? "true" : "false");
            ss << "\n";
            ss << AbstractMotorState::str();

            return ss.str();
        }

    } // namespace model
} // namespace common

