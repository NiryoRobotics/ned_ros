/*
    tools_interface_service_client.cpp
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

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>

#include "common/model/tool_state.hpp"

#include "tools_interface/tools_interface_core.hpp"

#include "ttl_driver/ttl_interface_core.hpp"

#include <XmlRpcValue.h>
#include <iostream>

static std::unique_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, pingTool)
{
    auto client = nh->serviceClient<tools_interface::PingDxlTool>("/niryo_robot/tools/ping_and_set_dxl_tool");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    tools_interface::PingDxlTool srv;
    client.call(srv);

    // surprisingly we must first create a variable to have it work
    int res = common::model::ToolState::TOOL_STATE_PING_OK;
    EXPECT_EQ(srv.response.state, res);
}

class ToolTestSuite : public ::testing::Test
{
    protected:
        static void SetUpTestCase()
        {
            pingTool();
        }

        static void pingTool()
        {
            auto client = nh->serviceClient<tools_interface::PingDxlTool>("/niryo_robot/tools/ping_and_set_dxl_tool");

            bool exists(client.waitForExistence(ros::Duration(1)));
            EXPECT_TRUE(exists);

            tools_interface::PingDxlTool srv_ping;
            client.call(srv_ping);

            id = srv_ping.response.id;
        }

        static int id;
};

int ToolTestSuite::id;

TEST_F(ToolTestSuite, openTool)
{
    if (id == 0) return;

    tools_interface::OpenGripper srv;

    XmlRpc::XmlRpcValue filters;

    nh->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = id;
            srv.request.open_position = static_cast<int>(filters[i]["specs"]["open_position"]);
            srv.request.open_speed = static_cast<int>(filters[i]["specs"]["open_speed"]);
            srv.request.open_hold_torque = static_cast<int>(filters[i]["specs"]["open_hold_torque"]);
            break;
        }
    }

    auto client = nh->serviceClient<tools_interface::OpenGripper>("/niryo_robot/tools/open_gripper");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);
    client.call(srv);

    int res = common::model::ToolState::GRIPPER_STATE_OPEN;
    EXPECT_EQ(srv.response.state, res);
}

TEST_F(ToolTestSuite, CloseTool)
{
    if (id == 0) return;

    tools_interface::CloseGripper srv;

    XmlRpc::XmlRpcValue filters;
    nh->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = id;
            srv.request.close_position = static_cast<int>(filters[i]["specs"]["close_position"]);
            srv.request.close_hold_torque = static_cast<int>(filters[i]["specs"]["close_hold_torque"]);
            srv.request.close_max_torque = static_cast<int>(filters[i]["specs"]["close_max_torque"]);
            srv.request.close_speed = static_cast<int>(filters[i]["specs"]["close_speed"]);
            break;
        }
    }

    auto client = nh->serviceClient<tools_interface::CloseGripper>("/niryo_robot/tools/close_gripper");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    client.call(srv);

    int res = common::model::ToolState::GRIPPER_STATE_CLOSE;
    EXPECT_EQ(srv.response.state, res);
}

TEST_F(ToolTestSuite, PullAirVacuumPump)
{
    if (id != 31) return;
    tools_interface::PullAirVacuumPump srv;

    XmlRpc::XmlRpcValue filters;
    nh->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = id;
            srv.request.pull_air_position = static_cast<int>(filters[i]["specs"]["pull_air_position"]);
            srv.request.pull_air_hold_torque = static_cast<int>(filters[i]["specs"]["pull_air_hold_torque"]);
            break;
        }
    }

    auto client = nh->serviceClient<tools_interface::PullAirVacuumPump>("/niryo_robot/tools/pull_air_vacuum_pump");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    client.call(srv);

    int res = common::model::ToolState::VACUUM_PUMP_STATE_PULLED;
    EXPECT_EQ(srv.response.state, res);
}

TEST_F(ToolTestSuite, PushAirVacuumPump)
{
    if (id != 31) return;
    tools_interface::PushAirVacuumPump srv;

    XmlRpc::XmlRpcValue filters;
    nh->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = id;
            srv.request.push_air_position = static_cast<int>(filters[i]["specs"]["push_air_position"]);
            break;
        }
    }

    auto client = nh->serviceClient<tools_interface::PushAirVacuumPump>("/niryo_robot/tools/push_air_vacuum_pump");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    client.call(srv);

    int res = common::model::ToolState::VACUUM_PUMP_STATE_PUSHED;
    EXPECT_EQ(srv.response.state, res);
}

TEST_F(ToolTestSuite, ToolReboot)
{   
    if (id != 0) return;

    auto client = nh->serviceClient<std_srvs::Trigger>("/niryo_robot/tools/reboot");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    std_srvs::Trigger srv;
    client.call(srv);

    EXPECT_EQ(srv.response.success, true);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tools_interface_service_client");

    nh = std::make_unique<ros::NodeHandle>("~");

    testing::InitGoogleTest(&argc, argv);

    ros::Duration(5.0).sleep();
    return RUN_ALL_TESTS();
}
