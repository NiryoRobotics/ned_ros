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
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef TOOLS_INTERFACE_CORE_HPP
#define TOOLS_INTERFACE_CORE_HPP

// c++
#include <memory>
#include <vector>
#include <mutex>
#include <map>

#include <ros/ros.h>

// niryo
#include "common/util/i_interface_core.hpp"

#include "common/model/tool_state.hpp"
#include "niryo_robot_msgs/Trigger.h"
#include "ttl_driver/ttl_interface_core.hpp"
#include "tools_interface/PingDxlTool.h"
#include "tools_interface/ToolCommand.h"
#include "tools_interface/Tool.h"

#include "std_msgs/Int32.h"

using ::common::model::EHardwareType;

namespace tools_interface
{
    /**
     * @brief The ToolsInterfaceCore class
     */
    class ToolsInterfaceCore : public common::util::IInterfaceCore
    {
    public:
        ToolsInterfaceCore(ros::NodeHandle &nh,
                           std::shared_ptr<ttl_driver::TtlInterfaceCore> ttl_interface);
        ~ToolsInterfaceCore() override = default;

        // non copyable class
        ToolsInterfaceCore(const ToolsInterfaceCore &) = delete;
        ToolsInterfaceCore(ToolsInterfaceCore &&) = delete;

        ToolsInterfaceCore &operator=(ToolsInterfaceCore &&) = delete;
        ToolsInterfaceCore &operator=(const ToolsInterfaceCore &) = delete;

        bool init(ros::NodeHandle &nh) override;

        bool rebootHardware(bool torque_on = true);

        bool isInitialized();

        // getters
        std::shared_ptr<common::model::ToolState> getToolState() const;

    private:
        struct ToolParams
        {
            int velocity_profile{0};
            int acceleration_profile{0};
            std::map<std::string, uint32_t> pid{{"p", 0}, {"i", 0}, {"d", 0}};
        };

        struct ToolConfig
        {
            std::string name;
            common::model::EHardwareType type;
            ToolParams params;
        };

        void initParameters(ros::NodeHandle &nh) override;
        void startServices(ros::NodeHandle &nh) override;
        void startPublishers(ros::NodeHandle &nh) override;
        void startSubscribers(ros::NodeHandle &nh) override;

        int initHardware(bool torque_on, uint8_t temperature_limit, uint8_t shutdown_configuration, const ToolConfig &tool_config);

        bool _callbackPingAndSetTool(tools_interface::PingDxlTool::Request &, tools_interface::PingDxlTool::Response &res);

        bool _callbackOpenGripper(tools_interface::ToolCommand::Request &req, tools_interface::ToolCommand::Response &res);
        bool _callbackCloseGripper(tools_interface::ToolCommand::Request &req, tools_interface::ToolCommand::Response &res);

        bool _callbackToolReboot(niryo_robot_msgs::Trigger::Request &, niryo_robot_msgs::Trigger::Response &res);

        bool _callbackPullAirVacuumPump(tools_interface::ToolCommand::Request &req, tools_interface::ToolCommand::Response &res);
        bool _callbackPushAirVacuumPump(tools_interface::ToolCommand::Request &req, tools_interface::ToolCommand::Response &res);

        void _toolCommand(uint32_t position, int torque, uint32_t velocity);
        void _publishToolConnection(const ros::TimerEvent &);

        void _waitForToolStop(int id, int timeout);

    private:
        int _temperature_limit{60};
        int _shutdown_configuration{53};
        int _vacuum_pump_timeout{3};
        int _gripper_timeout{3};

        std::mutex _tool_mutex;

        ros::Publisher _tool_connection_publisher;
        ros::Timer _tool_connection_publisher_timer;
        ros::Duration _tool_connection_publisher_duration{1.0};
        uint8_t _tool_ping_failed_cnt{0};

        std::shared_ptr<ttl_driver::TtlInterfaceCore> _ttl_interface;

        ros::ServiceServer _ping_and_set_dxl_tool_server;
        ros::ServiceServer _open_gripper_server;
        ros::ServiceServer _close_gripper_server;
        ros::ServiceServer _tool_reboot_server;
        ros::ServiceServer _pull_air_vacuum_pump_server;
        ros::ServiceServer _push_air_vacuum_pump_server;

        std::shared_ptr<common::model::ToolState> _toolState;
        std::map<uint8_t, ToolConfig> _available_tools_map;
        
    };

    /**
     * @brief ToolsInterfaceCore::getToolState
     * @return
     */
    inline std::shared_ptr<common::model::ToolState>
    ToolsInterfaceCore::getToolState() const
    {
        return _toolState;
    }

} // ToolsInterface

#endif
