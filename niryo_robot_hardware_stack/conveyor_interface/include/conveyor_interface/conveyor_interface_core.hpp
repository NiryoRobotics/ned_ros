/*
conveyor_interface_interface_core.hpp
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

#ifndef CONVEYOR_INTERFACE_CORE_HPP
#define CONVEYOR_INTERFACE_CORE_HPP

// c++
#include <memory>
#include <vector>

// ros
#include <ros/ros.h>

// niryo
#include "can_driver/can_interface_core.hpp"

#include "conveyor_interface/SetConveyor.h"
#include "conveyor_interface/ControlConveyor.h"
#include "conveyor_interface/ConveyorFeedbackArray.h"
#include "niryo_robot_msgs/CommandStatus.h"

#include "common/model/i_interface_core.hpp"

namespace conveyor_interface
{

/**
 * @brief The ConveyorInterfaceCore class
 */
class ConveyorInterfaceCore : public common::model::IInterfaceCore
{
    public:
        ConveyorInterfaceCore(ros::NodeHandle& nh, std::shared_ptr<common::model::IDriverCore> conveyor_driver);
        virtual ~ConveyorInterfaceCore() override;
        virtual bool init(ros::NodeHandle& nh) override;

        bool isInitialized();

    private:
        virtual void initParameters(ros::NodeHandle& nh) override;
        virtual void startServices(ros::NodeHandle& nh) override;
        virtual void startPublishers(ros::NodeHandle& nh) override;
        virtual void startSubscribers(ros::NodeHandle& nh) override;

        conveyor_interface::SetConveyor::Response addConveyor();
        conveyor_interface::SetConveyor::Response removeConveyor(uint8_t id);

        bool _callbackPingAndSetConveyor(conveyor_interface::SetConveyor::Request &req, conveyor_interface::SetConveyor::Response &res);
        bool _callbackControlConveyor(conveyor_interface::ControlConveyor::Request &req, conveyor_interface::ControlConveyor::Response &res);

        void _publishConveyorsFeedback();

    private:
        std::thread _publish_conveyors_feedback_thread;

        std::shared_ptr<common::model::IDriverCore> _conveyor_driver;

        ros::ServiceServer _ping_and_set_stepper_server;
        ros::ServiceServer _control_conveyor_server;

        ros::Publisher _conveyors_feedback_publisher;
        ros::Publisher _conveyor_status_publisher;

        // default id to look for to know if we have a stepper
        int _default_conveyor_id{6};

        // pool of possible id we can set for a newly connected conveyor
        std::set<uint8_t> _conveyor_pool_id_list;

        // list of currently connected conveyors
        std::vector<uint8_t> _current_conveyor_id_list;

        int _conveyor_max_effort{0};
        double _publish_feedback_frequency{0.0};
};
} // ConveyorInterface

#endif
