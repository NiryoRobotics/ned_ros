/*
iobject.hpp
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

#ifndef I_OBJECT_H
#define I_OBJECT_H

#include <string>

namespace common
{
namespace model
{

/**
 * @brief The IObject class is a base class for all objects in the model hierarchy
 */
class IObject
{
public:
    virtual ~IObject() = default;

    virtual void reset() = 0;
    virtual std::string str() const = 0;
    virtual bool isValid() const = 0;

protected:
    IObject() = default;
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c21-if-you-define-or-delete-any-copy-move-or-destructor-function-define-or-delete-them-all
    IObject( const IObject& ) = default;
    IObject( IObject&& ) = default;

    IObject& operator= ( IObject && ) = default;
    IObject& operator= ( const IObject& ) = default;
};

} // namespace model
} // namespace common

#endif // I_OBJECT_H
