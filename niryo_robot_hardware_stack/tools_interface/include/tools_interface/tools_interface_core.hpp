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

// according to xl-320 datasheet : 1 speed ~ 0.111 rpm ~ 1.8944 dxl position per second
#define XL320_STEPS_FOR_1_SPEED 1.8944 // 0.111 * 1024 / 60

// according to xl-330 datasheet : 1 speed ~ 0.229 rpm ~ 3.9083 dxl position per second
#define XL330_STEPS_FOR_1_SPEED  15.6331 // 0.229 * 4096 / 60

#include <memory>

#include <ros/ros.h>
#include <vector>

#include "tools_interface/tool_state.hpp"

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

            void initParams();
            void initServices();

            void pubToolId(int id);

        private:
            ros::NodeHandle _nh;
            std::shared_ptr<DynamixelDriver::DynamixelDriverCore> _dynamixel;
            ToolState _toolState;
            std::mutex _tool_mutex;

            void _checkToolConnection();
            std::shared_ptr<std::thread> _check_tool_connection_thread;
            double _check_tool_connection_frequency;

            ros::ServiceServer _ping_and_set_dxl_tool_server;
            ros::ServiceServer _open_gripper_server;
            ros::ServiceServer _close_gripper_server;
            ros::ServiceServer _pull_air_vacuum_pump_server;
            ros::ServiceServer _push_air_vacuum_pump_server;

            ros::Publisher _current_tools_id_publisher;

            std::map<uint8_t, DynamixelDriver::DxlMotorType_t> _available_tools_map;

            bool _callbackPingAndSetDxlTool(tools_interface::PingDxlTool::Request &req, tools_interface::PingDxlTool::Response &res);

            bool _callbackOpenGripper(tools_interface::OpenGripper::Request &req, tools_interface::OpenGripper::Response &res);
            bool _callbackCloseGripper(tools_interface::CloseGripper::Request &req, tools_interface::CloseGripper::Response &res);

            bool _callbackPullAirVacuumPump(tools_interface::PullAirVacuumPump::Request &req, tools_interface::PullAirVacuumPump::Response &res);
            bool _callbackPushAirVacuumPump(tools_interface::PushAirVacuumPump::Request &req, tools_interface::PushAirVacuumPump::Response &res);
    };
} // ToolsInterface

#endif
