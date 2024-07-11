/*
abstract_enum.hpp
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

#ifndef ABSTRACT_ENUM_H
#define ABSTRACT_ENUM_H

#include <string>
#include <vector>
#include <map>
#include <algorithm> // std::sort
#include <stdexcept>

namespace common
{
    namespace model
    {

        /**
         * @brief The AbstractEnum class
         */
        template <class C, typename E>
        class AbstractEnum
        {
        public:
            operator E() const { return _enum; }

            const std::string &toString() const;

            std::vector<std::string> values(bool sort = false) const;

        protected:
            AbstractEnum(E e = static_cast<E>(0))
                : _enum(e)
            {
            }

            AbstractEnum(const std::string &s)
            {
                fromString(s.c_str());
            }

            AbstractEnum(const char *const str)
            {
                fromString(str);
            }

        private:
            void fromString(const char *str);

            // use curiously recurring template pattern  (CRTP)
            static std::map<E, std::string> initialize()
            {
                return C::initialize();
            }

            E _enum;
            static std::map<E, std::string> _map;
        };

        /**
         * @brief AbstractEnum<C, E>::_map
         */
        template <class C, typename E>
        std::map<E, std::string> AbstractEnum<C, E>::_map = initialize();

        /**
         * @brief AbstractEnum<C, E>::fromString
         * @param str
         */
        template <class C, typename E>
        void AbstractEnum<C, E>::fromString(const char *const str)
        {
            _enum = static_cast<E>(0); //
            if (str)
            {
                typename std::map<E, std::string>::const_iterator iter = _map.begin();
                for (; iter != _map.end(); ++iter)
                {
                    if (iter->second == str)
                    {
                        _enum = iter->first;
                        break;
                    }
                }
                if (iter == _map.end())
                    throw std::out_of_range("");
            }
        }

        /**
         * @brief AbstractEnum<C, E>::toString
         * @return
         */
        template <class C, typename E>
        const std::string &
        AbstractEnum<C, E>::toString() const
        {
            typename std::map<E, std::string>::const_iterator iter = _map.find(_enum);
            if (iter == _map.end())
                throw std::out_of_range("");
            return iter->second;
        }

        /**
         * @brief AbstractEnum<C, E>::values
         * @param sort
         * @return
         */
        template <class C, typename E>
        std::vector<std::string>
        AbstractEnum<C, E>::values(bool sort) const
        {
            std::vector<std::string> values;
            for (typename std::map<E, std::string>::const_iterator iter = _map.begin();
                 iter != _map.end();
                 ++iter)
            {
                values.push_back(iter->second);
            }
            if (sort)
                std::sort(values.begin(), values.end());

            return values;
        }

    } // model
} // common

#endif // ABSTRACT_ENUM_H
