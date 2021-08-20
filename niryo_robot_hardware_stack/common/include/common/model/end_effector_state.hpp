/*
end_effector_state.hpp
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

#ifndef END_EFFECTOR_STATE_HPP
#define END_EFFECTOR_STATE_HPP

#include "abstract_hardware_state.hpp"

#include <stdint.h>
#include <string>

#include "hardware_type_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The EndEffectorState class
 */
class EndEffectorState : public AbstractHardwareState
{
    public:
        enum class EActionType
        {
            HANDLE_HELD_ACTION,
            LONG_PUSH_ACTION,
            SINGLE_PUSH_ACTION,
            DOUBLE_PUSH_ACTION
        };

    public:
        EndEffectorState();
        EndEffectorState(uint8_t id);

        virtual ~EndEffectorState() override;

        // AbstractHardwareState interface
        virtual std::string str() const override;

        // IObject interface
    public:
        virtual bool isValid() const override;
};

} // namespace model
} // namespace common

#endif // END_EFFECTOR_STATE_HPP
