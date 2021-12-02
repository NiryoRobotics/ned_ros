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
#include <string>

static std::unique_ptr<ros::NodeHandle> nh;

TEST(ToolTestConfigSuite, correctPath)
{
    ros::NodeHandle nh("");
    ASSERT_TRUE(nh.hasParam("tools_interface/tools_params/id_list"));
    ASSERT_TRUE(nh.hasParam("tools_interface/tools_params/type_list"));
    ASSERT_TRUE(nh.hasParam("tools_interface/tools_params/name_list"));
    ASSERT_TRUE(nh.hasParam("tools_interface/check_tool_connection_frequency"));
}

TEST(ToolTestConfigSuite, correctSize)
{
    ros::NodeHandle nh("");
    std::vector<int> idList;
    std::vector<std::string> typeList;
    std::vector<std::string> nameList;

    nh.getParam("tools_interface/tools_params/id_list", idList);
    nh.getParam("tools_interface/tools_params/type_list", typeList);
    nh.getParam("tools_interface/tools_params/name_list", nameList);

    ASSERT_TRUE(idList.size() == typeList.size());
    ASSERT_TRUE(idList.size() == nameList.size());
}

TEST(ToolTestSetTool, addTool)
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

class ToolTestControlSuite : public ::testing::Test
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

            // Continue only if set tool successfully 
            int res = common::model::ToolState::TOOL_STATE_PING_OK;
            ASSERT_EQ(srv_ping.response.state, res);

            id = srv_ping.response.tool.id;
            ROS_ERROR("TEST id %d, res %d", id, (int)srv_ping.response.state);
        }

        static int id;
};

int ToolTestControlSuite::id;

/**
 * @brief Test check if tool scanned is in list id config
 */
TEST_F(ToolTestControlSuite, checkToolScannedId)
{
    ros::NodeHandle nh("");
    std::vector<int> id_list;
    nh.getParam("tools_interface/tools_params/id_list", id_list);
    for (auto i : id_list)
    {
        ROS_ERROR("TEST id list %d", i);
    }
    ROS_ERROR("TEST id %d", id);
    EXPECT_TRUE(std::find(id_list.begin(), id_list.end(), id) != id_list.end()); 
}

/**
 * @brief Test check if add Tool fail
 */


TEST_F(ToolTestControlSuite, openTool)
{
    ASSERT_FALSE(id == -1);

    tools_interface::ToolCommand srv;

    XmlRpc::XmlRpcValue filters;

    nh->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = static_cast<uint8_t>(id);
            srv.request.position = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["open_position"]));
            srv.request.speed = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["open_speed"]));
            srv.request.hold_torque = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["open_hold_torque"]));
            srv.request.max_torque = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["open_max_torque"]));
        }
    }

    auto client = nh->serviceClient<tools_interface::ToolCommand>("/niryo_robot/tools/open_gripper");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);
    client.call(srv);

    int res = common::model::ToolState::GRIPPER_STATE_OPEN;
    EXPECT_EQ(srv.response.state, res);
}

TEST_F(ToolTestControlSuite, CloseTool)
{
    ASSERT_FALSE(id == -1);

    tools_interface::ToolCommand srv;

    XmlRpc::XmlRpcValue filters;
    nh->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = static_cast<uint8_t>(id);
            srv.request.position = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["close_position"]));
            srv.request.speed = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["close_speed"]));
            srv.request.hold_torque = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["close_hold_torque"]));
            srv.request.max_torque = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["close_max_torque"]));
            break;
        }
    }

    auto client = nh->serviceClient<tools_interface::ToolCommand>("/niryo_robot/tools/close_gripper");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    client.call(srv);

    int res = common::model::ToolState::GRIPPER_STATE_CLOSE;
    EXPECT_EQ(srv.response.state, res);
}

TEST_F(ToolTestControlSuite, PullAirVacuumPump)
{
    if (id != 31) return;
    tools_interface::ToolCommand srv;

    XmlRpc::XmlRpcValue filters;
    nh->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = static_cast<uint8_t>(id);
            srv.request.position = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["pull_air_position"]));
            srv.request.hold_torque = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["pull_air_hold_torque"]));
            srv.request.speed = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["pull_air_velocity"]));
            srv.request.max_torque = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["pull_air_max_torque"]));
            break;
        }
    }

    auto client = nh->serviceClient<tools_interface::ToolCommand>("/niryo_robot/tools/pull_air_vacuum_pump");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    client.call(srv);

    int res = common::model::ToolState::VACUUM_PUMP_STATE_PULLED;
    EXPECT_EQ(srv.response.state, res);
}

TEST_F(ToolTestControlSuite, PushAirVacuumPump)
{
    if (id != 31) return;
    tools_interface::ToolCommand srv;

    XmlRpc::XmlRpcValue filters;
    nh->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = static_cast<uint8_t>(id);
            srv.request.position = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["push_air_position"]));
            srv.request.speed = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["push_air_velocity"]));
            srv.request.max_torque = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["push_air_max_torque"]));
            break;
        }
    }

    auto client = nh->serviceClient<tools_interface::ToolCommand>("/niryo_robot/tools/push_air_vacuum_pump");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    client.call(srv);

    int res = common::model::ToolState::VACUUM_PUMP_STATE_PUSHED;
    EXPECT_EQ(srv.response.state, res);
}

TEST_F(ToolTestControlSuite, ToolReboot)
{
    ASSERT_FALSE(id == -1);

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
