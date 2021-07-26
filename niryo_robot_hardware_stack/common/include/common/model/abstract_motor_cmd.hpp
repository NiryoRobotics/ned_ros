/*
abstract_motor_cmd.hpp
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
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef ABSTRACT_MOTOR_CMD_H
#define ABSTRACT_MOTOR_CMD_H

#include <string>
#include <type_traits>

#include "common/model/iobject.hpp"
#include "common/model/abstract_enum.hpp"

namespace common
{
namespace model
{

/**
 * @brief The AbstractMotorCmd class
 */
template<typename E>
class AbstractMotorCmd : public IObject
{

    public:
        AbstractMotorCmd(E type);
        virtual ~AbstractMotorCmd();

        virtual void reset() = 0;
        virtual void clear() = 0;

        // setters
        void setType(E type);

        // getters
        E getType() const;

    protected:
        E _type;

    private:
        AbstractMotorCmd() = delete;

};

/**
 * @brief AbstractMotorCmd<E>::AbstractMotorCmd
 * @param type
 */
template<typename E>
AbstractMotorCmd<E>::AbstractMotorCmd(E type)
{
    static_assert(std::is_enum<E>::value, "E must be an enum");
    _type = type;
}

/**
 * @brief AbstractMotorCmd<E>::~AbstractMotorCmd
 */
template<typename E>
AbstractMotorCmd<E>::~AbstractMotorCmd()
{
}

/**
 * @brief AbstractMotorCmd<E>::setType
 * @param type
 */
template<typename E>
void AbstractMotorCmd<E>::setType(E type)
{
    _type = type;
}

/**
 * @brief AbstractMotorCmd<E>::getType
 * @return
 */
template<typename E>
E AbstractMotorCmd<E>::getType() const
{
    return _type;
}

} // namespace model
} // namespace common

#endif
