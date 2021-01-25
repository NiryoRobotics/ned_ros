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

#include <stdint.h>
#include <string>

class JointState {

    public:
        JointState(std::string name, uint8_t type, uint8_t id );

        std::string& getName();
        void setName(std::string &name);

        uint8_t getId();
        void setId(uint8_t id);

        uint8_t getType();
        void setType(uint8_t id);

        bool needCalibration();
        void setNeedCalibration(bool need_calibration);

        void setPosition(double position);
        double getPosition();

        bool operator==(const JointState& other) const;

    protected : 

        std::string _name;
        uint8_t _id;
        uint8_t _type;

        double _position;
        bool _need_calibration;

};

#endif