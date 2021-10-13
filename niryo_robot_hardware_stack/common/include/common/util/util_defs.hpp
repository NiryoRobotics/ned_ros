/*
util_defs.hpp
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

#ifndef UTIL_DEFS_H
#define UTIL_DEFS_H
// C++

/*
common_defs.h
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

// std
#include <future>
#include <type_traits>
#include <memory>

// Common
#include "common/common_defs.hpp"

namespace common
{
namespace util
{

/**
 * Non copyable class
 */
template <typename T>
class CNonCopyable
{
protected:
    CNonCopyable () = default;
    ~CNonCopyable () = default; // / Protected non-virtual destructor

public:
    CNonCopyable( const CNonCopyable& ) = delete;
    CNonCopyable& operator=( const CNonCopyable& ) = delete;
    CNonCopyable( CNonCopyable&& ) = delete;
    CNonCopyable& operator=( CNonCopyable&& ) = delete;

    T& operator= ( const T & ) = delete;
    T& operator= ( T && ) = delete;
};

/**
 * equivalent de std::async mais en certifiant que la tache sera réellement parallelisée
 */
template<typename F, typename... Ts>
inline
std::future<typename std::result_of<F(Ts...)>::type>
reallyAsync(F&& f, Ts&&... params)      // return future for asynchronous call to f(params...)
{
    return std::async(std::launch::async,
                      std::forward<F>(f),
                      std::forward<Ts>(params)...);
}

} // util
} // common


#endif // UTIL_DEFS_H


