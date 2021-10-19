/*
unique_ptr_cast.hpp
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

#ifndef UNIQUE_PTR_CAST
#define UNIQUE_PTR_CAST

#include <memory>

namespace common
{
namespace util
{

template<typename Derived, typename Base>
static std::unique_ptr<Derived> static_unique_ptr_cast(std::unique_ptr<Base>&& p);

template<typename Derived, typename Base>
static std::unique_ptr<Derived> dynamic_unique_ptr_cast(std::unique_ptr<Base>&& p);


template<typename Derived, typename Base>
std::unique_ptr<Derived> static_unique_ptr_cast(std::unique_ptr<Base>&& p)
{
    auto d = static_cast<Derived *>(p.release());
    return std::unique_ptr<Derived>(d);
}

template<typename Derived, typename Base>
std::unique_ptr<Derived> dynamic_unique_ptr_cast(std::unique_ptr<Base>&& p)
{
    if (auto *result = dynamic_cast<Derived *>(p.get()))
    {
        p.release();
        return std::unique_ptr<Derived>(result);
    }
    return std::unique_ptr<Derived>(nullptr);
}
}   // namespace util
}   // namespace common
#endif  // UNIQUE_PTR_CAST
