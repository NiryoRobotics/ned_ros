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

#include "common/model/joint_state.hpp"
#include <sstream>

using namespace std;

namespace common {
    namespace model {

        /**
         * @brief JointState::JointState
         */
        JointState::JointState() :
            AbstractMotorState()
        {
        }

        /**
         * @brief JointState::JointState
         * @param name
         * @param type
         * @param id
         */
        JointState::JointState(std::string name, EMotorType type, uint8_t id) :
            AbstractMotorState(id, type),
            _name(name)
        {
        }

        /**
         * @brief JointState::~JointState
         */
        JointState::~JointState()
        {

        }

        /**
         * @brief JointState::operator ==
         * @param m
         * @return
         */
        bool JointState::operator==(const JointState& m) const
        {
            return((this->_type == m._type) && (this->_id == m._id));
        }

        /**
         * @brief JointState::setName
         * @param name
         */
        void JointState::setName(string& name)
        {
            _name = name;
        }

        /**
         * @brief JointState::setOffsetPosition
         * @param offset_position
         */
        void JointState::setOffsetPosition(double offset_position)
        {
            _offset_position = offset_position;
        }

        //***********************
        //  AbstractMotor intf
        //***********************

        /**
         * @brief JointState::reset
         */
        void JointState::reset()
        {
            AbstractMotorState::reset();
            _name.clear();
            _need_calibration = false;
        }

        /**
         * @brief common::model::JointState::isValid
         * @return
         */
        bool common::model::JointState::isValid() const
        {
            return (0 != getId() && EMotorType::UNKNOWN != getType());
        }

        /**
         * @brief JointState::str
         * @return
         */
        string JointState::str() const
        {
            ostringstream ss;

            ss << "JointState : ";
            ss << "\n---\n";
            ss << "type: " << MotorTypeEnum(_type).toString() << ", ";
            ss << "name: " << "\"" << _name << "\"" << ", ";
            ss << "position: " << pos << ", ";
            ss << "need calibration: " << (_need_calibration ? "true" : "false") << ", ";
            ss << "pos(" << pos << "), ";
            ss << "cmd(" << cmd << "), ";
            ss << "vel(" << vel << "), ";
            ss << "eff(" << eff << ")";
            ss << "\n";
            ss << AbstractMotorState::str();

            return ss.str();
        }

    } // namespace model
} // namespace common

