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
#include "ttl_driver/end_effector_driver.hpp"

namespace end_effector_interface
{

/**
 * @brief The EndEffectorInterfaceCore class
 */
class EndEffectorInterfaceCore : public common::model::IInterfaceCore
{
    enum class EActionType
    {
        eHandHeldAction,
        eLongPushAction,
        eSinglePushAction,
        eDoublePushAction
    };

    public:
    // TODO(CC) What is the best param to give ?
    // TtlInterfaceCore ? -> used for single and sync cmd queues, not really the case here
    // TtlManager ? -> used to manager motors, not really the case here
    // packetHandler and portHandler ? -> then implement our own EEManager and our own eeDriver
    // eeDriver ? -> then instanciate eeDriver in EEManager, can be usefull for HW status of all TTL components.
    // but it is yet another stuff in ttl driver
        EndEffectorInterfaceCore(ros::NodeHandle& nh,
                                 std::shared_ptr<ttl_driver::EndEffectorDriver<ttl_driver::EndEffectorReg> > ee_driver);
        virtual ~EndEffectorInterfaceCore() override;

        virtual bool init(ros::NodeHandle &nh) override;

    private:
        virtual void initParameters(ros::NodeHandle& nh) override;
        virtual void startServices(ros::NodeHandle& nh) override;
        virtual void startPublishers(ros::NodeHandle& nh) override;
        virtual void startSubscribers(ros::NodeHandle& nh) override;

    private:
        std::shared_ptr<ttl_driver::EndEffectorDriver<ttl_driver::EndEffectorReg> > _ee_driver;

        uint8_t _id;
};
} // EndEffectorInterface

#endif // END_EFFECTOR_INTERFACE_CORE_HPP
