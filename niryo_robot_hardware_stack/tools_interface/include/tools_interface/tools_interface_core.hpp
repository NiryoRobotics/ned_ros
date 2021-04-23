/*
    tools_interface_core.hpp
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

#ifndef TOOLS_INTERFACE_CORE_HPP
#define TOOLS_INTERFACE_CORE_HPP



#include <memory>

#include <ros/ros.h>
#include <vector>

#include "model/tool_state.hpp"

#include "tools_interface/PingDxlTool.h"
#include "tools_interface/OpenGripper.h"
#include "tools_interface/CloseGripper.h"
#include "tools_interface/PullAirVacuumPump.h"
#include "tools_interface/PushAirVacuumPump.h"

#include "dynamixel_driver/dxl_driver_core.hpp"

#include "std_msgs/Int32.h"

namespace ToolsInterface {

    class ToolsInterfaceCore
    {
        public:

            ToolsInterfaceCore(std::shared_ptr<DynamixelDriver::DynamixelDriverCore> dynamixel);
            virtual ~ToolsInterfaceCore();

            void pubToolId(int id);

        private:
            void initParams();
            void initServices();

        private:
            ros::NodeHandle _nh;
            std::shared_ptr<DynamixelDriver::DynamixelDriverCore> _dynamixel;
            common::model::ToolState _toolState;
            std::mutex _tool_mutex;

            std::thread _check_tool_connection_thread;

            void _checkToolConnection();
            double _check_tool_connection_frequency;

            ros::ServiceServer _ping_and_set_dxl_tool_server;
            ros::ServiceServer _open_gripper_server;
            ros::ServiceServer _close_gripper_server;
            ros::ServiceServer _pull_air_vacuum_pump_server;
            ros::ServiceServer _push_air_vacuum_pump_server;

            ros::Publisher _current_tools_id_publisher;

            std::map<uint8_t, common::model::EMotorType> _available_tools_map;

            bool _callbackPingAndSetDxlTool(tools_interface::PingDxlTool::Request &, tools_interface::PingDxlTool::Response &res);

            bool _callbackOpenGripper(tools_interface::OpenGripper::Request &req, tools_interface::OpenGripper::Response &res);
            bool _callbackCloseGripper(tools_interface::CloseGripper::Request &req, tools_interface::CloseGripper::Response &res);

            bool _callbackPullAirVacuumPump(tools_interface::PullAirVacuumPump::Request &req, tools_interface::PullAirVacuumPump::Response &res);
            bool _callbackPushAirVacuumPump(tools_interface::PushAirVacuumPump::Request &req, tools_interface::PushAirVacuumPump::Response &res);
    };
} // ToolsInterface

#endif
