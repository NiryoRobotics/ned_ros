/*
tool_state.hpp
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

#ifndef TOOL_STATE_HPP
#define TOOL_STATE_HPP

#include <stdint.h>
#include <string>

#include "motor_type_enum.hpp"
#include "dxl_motor_state.hpp"

namespace common
{
namespace model
{

/**
 * @brief The ToolState class
 */
class ToolState : public DxlMotorState
{
    public:
        ToolState();
        ToolState(std::string name, EMotorType type, uint8_t id);

        virtual ~ToolState() override;


        void setName(std::string name);
        void setPosition(double position);

        std::string getToolName() const;
        double getPositionRef() const;

        bool isConnected() const;

        // DxlMotorState interface
        virtual void reset() override;
        virtual std::string str() const override;

    public:
        static constexpr int TOOL_STATE_PING_OK       = 0x01;
        static constexpr int TOOL_STATE_PING_ERROR    = 0x02;
        static constexpr int TOOL_STATE_WRONG_ID      = 0x03;
        static constexpr int TOOL_STATE_TIMEOUT       = 0x04;

        static constexpr int GRIPPER_STATE_OPEN       = 0x10;
        static constexpr int GRIPPER_STATE_CLOSE      = 0x11;

        static constexpr int VACUUM_PUMP_STATE_PULLED = 0x20;
        static constexpr int VACUUM_PUMP_STATE_PUSHED = 0x21;

    protected:
        // use joint _name ?
        std::string _tool_name;

        bool _connected;
        double _position;
};

/**
 * @brief ToolState::getName
 * @return
 */
inline
std::string ToolState::getToolName() const
{
    return _tool_name;
}

/**
 * @brief ToolState::getPositionRef
 * @return
 */
inline
double ToolState::getPositionRef() const
{
    return _position;
}

/**
 * @brief ToolState::isConnected
 * @return
 */
inline
bool ToolState::isConnected() const
{
    return _connected;
}

} // namespace model
} // namespace common

#endif // TOOL_STATE_HPP
