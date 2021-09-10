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

#include "common/model/end_effector_state.hpp"
#include "ttl_driver/ttl_interface_core.hpp"

namespace end_effector_interface
{

/**
 * @brief The EndEffectorInterfaceCore class manages the services and topics for the end effector.
 * The TTL management is done in ttl_manager
 * Every runtime critical operation is done in TtlManager
 */
class EndEffectorInterfaceCore : public common::model::IInterfaceCore
{

    public:
        EndEffectorInterfaceCore(ros::NodeHandle& nh,
                                 std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface);
        virtual ~EndEffectorInterfaceCore() override;

        virtual bool init(ros::NodeHandle &nh) override;

        // getters
        common::model::EndEffectorState getEndEffectorState() const;

    private:
        virtual void initParameters(ros::NodeHandle& nh) override;
        virtual void startServices(ros::NodeHandle& nh) override;
        virtual void startPublishers(ros::NodeHandle& nh) override;
        virtual void startSubscribers(ros::NodeHandle& nh) override;

        void initEndEffectorHardware();
        void _publishButtonState();

    private:
        std::mutex _buttons_status_mutex;

        std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttl_interface;

        ros::ServiceClient _learning_mode_client;

        ros::Publisher _free_drive_button_state_publisher;
        ros::Publisher _save_pos_button_state_publisher;
        ros::Publisher _custom_button_state_publisher;

        std::thread _publish_buttons_state_thread;

        common::model::EndEffectorState _end_effector_state;
        uint8_t _id;
        bool _is_learning_mode{false};

        double _check_end_effector_status_frequency{0.0};
};

common::model::EndEffectorState
EndEffectorInterfaceCore::getEndEffectorState() const
{
    return _end_effector_state;
}


} // EndEffectorInterface

#endif // END_EFFECTOR_INTERFACE_CORE_HPP
