/*
    idriver.hpp
    Copyright (C) 2017 Niryo
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

#ifndef IDRIVER_H
#define IDRIVER_H

#include <stdint.h>
#include <string>
#include <vector>
#include "joint_state.hpp"

namespace common {
    namespace model {

        class IDriver
        {
            public:
                virtual ~IDriver() = 0;
                virtual void removeMotor(uint8_t id) = 0;
                virtual bool isConnectionOk() const = 0;
                virtual int scanAndCheck() = 0;

                virtual size_t getNbMotors() const = 0;
                virtual void getBusState(bool& connection_state, std::vector<uint8_t>& motor_id, std::string& debug_msg) const = 0;
                virtual std::string getErrorMessage() const = 0;

            private:
                virtual bool init() = 0;
                virtual bool hasMotors() = 0;
        };

        inline
        IDriver::~IDriver()
        {

        }

    } // namespace model
} // namespace common

#endif
