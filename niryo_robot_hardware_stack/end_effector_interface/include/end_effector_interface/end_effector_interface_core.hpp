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
#include "common/util/i_interface_core.hpp"

#include "common/model/end_effector_state.hpp"
#include "ttl_driver/ttl_interface_core.hpp"

#include "end_effector_interface/EEButtonStatus.h"
#include "end_effector_interface/EEIOState.h"
#include "end_effector_interface/SetEEDigitalOut.h"

namespace end_effector_interface
{

/**
 * @brief The EndEffectorInterfaceCore class manages the services and topics for the end effector.
 * The TTL management is done in ttl_manager
 * Every runtime critical operation is done in TtlManager
 */
class EndEffectorInterfaceCore : public common::util::IInterfaceCore
{

    public:
        EndEffectorInterfaceCore(ros::NodeHandle& nh,
                                 std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface);
        ~EndEffectorInterfaceCore() override = default;

        // non copyable class
        EndEffectorInterfaceCore( const EndEffectorInterfaceCore& ) = delete;
        EndEffectorInterfaceCore( EndEffectorInterfaceCore&& ) = delete;

        EndEffectorInterfaceCore& operator= ( EndEffectorInterfaceCore && ) = delete;
        EndEffectorInterfaceCore& operator= ( const EndEffectorInterfaceCore& ) = delete;

        bool init(ros::NodeHandle &nh) override;
        bool rebootHardware();

        // getters
        std::shared_ptr<common::model::EndEffectorState> getEndEffectorState() const;

    private:
        void initParameters(ros::NodeHandle& nh) override;
        void startServices(ros::NodeHandle& nh) override;
        void startPublishers(ros::NodeHandle& nh) override;
        void startSubscribers(ros::NodeHandle& nh) override;

        void initEndEffectorHardware();
        int initHardware();
        void _publishButtonState(const ros::TimerEvent&);

        bool _callbackSetIOState(end_effector_interface::SetEEDigitalOut::Request &req,
                                 end_effector_interface::SetEEDigitalOut::Response &res);

    private:
        std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttl_interface;

        ros::Publisher _free_drive_button_state_publisher;
        ros::Publisher _save_pos_button_state_publisher;
        ros::Publisher _custom_button_state_publisher;

        ros::Publisher _digital_out_publisher;

        ros::Timer _states_publisher_timer;
        ros::Duration _states_publisher_duration{1.0};

        ros::ServiceServer _digital_in_server;

        std::shared_ptr<common::model::EndEffectorState> _end_effector_state;
        uint8_t _id{1};
        bool _simulation{false};
};

/**
 * @brief EndEffectorInterfaceCore::getEndEffectorState
 * @return
 */
inline
std::shared_ptr<common::model::EndEffectorState>
EndEffectorInterfaceCore::getEndEffectorState() const
{
    return _end_effector_state;
}


} // EndEffectorInterface

#endif // END_EFFECTOR_INTERFACE_CORE_HPP
