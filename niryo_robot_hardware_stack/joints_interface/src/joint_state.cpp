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

#include "joints_interface/joint_state.hpp"

JointState::JointState(std::string name, uint8_t type, uint8_t id) :
    _name(name), _id(id), _type(type)
{

}

bool JointState::operator==(const JointState& m) const
{
    return((this->_type == m._type) && (this->_id == m._id));
}

std::string& JointState::getName()
{
    return _name;
}

void JointState::setName(std::string& name)
{
    _name = name;
}


uint8_t JointState::getId()
{
    return _id;
}

void JointState::setId(uint8_t id)
{
    _id = id;
}

uint8_t JointState::getType()
{
    return _type;
}

void JointState::setType(uint8_t type)
{
    _type = type;
}

bool JointState::needCalibration()
{
    return _need_calibration;
}

void JointState::setNeedCalibration(bool need_calibration)
{
    _need_calibration = need_calibration;
}

void JointState::setPosition(double position)
{
    _position = position;
}

double JointState::getPosition()
{
    return _position;
}

