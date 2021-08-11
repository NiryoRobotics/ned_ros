/*
end_effector_interface_core.hpp
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

#ifndef END_EFFECTOR_INTERFACE_CORE_HPP
#define END_EFFECTOR_INTERFACE_CORE_HPP

// c++
#include <memory>
#include <vector>

#include <ros/ros.h>

// niryo
#include "common/model/i_interface_core.hpp"

#include "common/model/tool_state.hpp"
#include "ttl_driver/ttl_interface_core.hpp"

namespace end_effector_interface
{

/**
 * @brief The EndEffectorInterfaceCore class
 */
class EndEffectorInterfaceCore : public common::model::IInterfaceCore
{
    public:
        EndEffectorInterfaceCore(ros::NodeHandle& nh,
                           std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface);
        virtual ~EndEffectorInterfaceCore() override;

        virtual bool init(ros::NodeHandle &nh) override;

    private:
        virtual void initParameters(ros::NodeHandle& nh) override;
        virtual void startServices(ros::NodeHandle& nh) override;
        virtual void startPublishers(ros::NodeHandle& nh) override;
        virtual void startSubscribers(ros::NodeHandle& nh) override;

    private:
        std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttl_interface;

};
} // EndEffectorInterface

#endif // END_EFFECTOR_INTERFACE_CORE_HPP
