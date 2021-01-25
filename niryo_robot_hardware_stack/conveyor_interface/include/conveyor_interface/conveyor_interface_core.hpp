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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONVEYOR_INTERFACE_CORE_HPP
#define CONVEYOR_INTERFACE_CORE_HPP

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <vector>

#include "stepper_driver/stepper_driver_core.hpp"

#include "stepper_driver/StepperCmd.h"
#include "conveyor_interface/SetConveyor.h"
#include "conveyor_interface/ControlConveyor.h"
#include "conveyor_interface/ConveyorFeedbackArray.h"
#include "niryo_robot_msgs/CommandStatus.h"

class ConveyorInterfaceCore
{
    public:
        
        ConveyorInterfaceCore(boost::shared_ptr<StepperDriver::StepperDriverCore> &stepper);
        void initServices();
        void initParams();

    private:

        ros::NodeHandle _nh;
        boost::shared_ptr<StepperDriver::StepperDriverCore> &_stepper;

        ros::ServiceServer _ping_and_set_stepper_server;
        ros::ServiceServer _control_conveyor_server;
        ros::Publisher _conveyors_feedback_publisher;

        ros::Publisher _conveyor_status_publisher;
        boost::shared_ptr<std::thread> _publish_conveyor_status_thread;

        std::vector<uint8_t> _list_conveyor_id;
        std::vector<uint8_t> _list_available_id; 
        std::vector<int> _list_possible_conveyor_id;
        int _conveyor_id;
        int _conveyor_max_effort;
        double _publish_feedback_frequency;

        boost::shared_ptr<std::thread> _publish_conveyors_feedback_thread;

        bool _callbackPingAndSetConveyor(conveyor_interface::SetConveyor::Request &req, conveyor_interface::SetConveyor::Response &res);
        bool _callbackControlConveyor(conveyor_interface::ControlConveyor::Request &req, conveyor_interface::ControlConveyor::Response &res);

        void _publishConveyorsFeedback();
};
#endif
