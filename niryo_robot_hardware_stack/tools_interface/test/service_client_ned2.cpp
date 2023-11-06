/*
    tools_interface_service_client_ned2.cpp
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

#include "common/model/tool_state.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <vector>

#include "tools_interface/tools_interface_core.hpp"

#include "ttl_driver/ttl_interface_core.hpp"

#include <XmlRpcValue.h>
#include <memory>
#include <iostream>
#include <string>

static std::unique_ptr<ros::NodeHandle> nh_g;

/**
 * @brief Check config tool added correctly or not
 */
TEST(ToolTestConfigSuite, correctPath)
{
    ros::NodeHandle nh("");
    ASSERT_TRUE(nh.hasParam("tools_interface/tools_params/id_list"));
    ASSERT_TRUE(nh.hasParam("tools_interface/tools_params/type_list"));
    ASSERT_TRUE(nh.hasParam("tools_interface/tools_params/name_list"));
    ASSERT_TRUE(nh.hasParam("tools_interface/check_tool_connection_frequency"));
}

/**
 * @brief Test checks config does not miss any param
 *
 */
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

/**
 * @brief Check if config in tool interface and tool commander is compatible
 */
TEST(ToolTestConfigSuite, compabilityConfig)
{
    // config in tool interface
    ros::NodeHandle nh("");
    std::vector<int> idList;
    nh.getParam("tools_interface/tools_params/id_list", idList);

    tools_interface::ToolCommand srv;
    XmlRpc::XmlRpcValue filters;

    nh_g->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        int id = static_cast<int>(filters[i]["id"]);
        if (id == 0)
            continue;
        EXPECT_TRUE(std::find(idList.begin(), idList.end(), id) != idList.end());
    }
}

/**
 * @brief Test checks if can add tool correctly if tool connected physically
 */
TEST(ToolTestSetTool, addTool)
{
    ros::NodeHandle nh("");
    std::vector<int> idList;
    nh.getParam("tools_interface/tools_params/id_list", idList);

    auto client = nh_g->serviceClient<tools_interface::PingDxlTool>("/niryo_robot/tools/ping_and_set_dxl_tool");

    // using timeout 5 seconds for the first calling service Ros to delay tests until tools_interface node launched
    bool exists(client.waitForExistence(ros::Duration(5)));
    EXPECT_TRUE(exists);

    tools_interface::PingDxlTool srv;
    client.call(srv);

    // surprisingly we must first create a variable to have it work
    int res = common::model::ToolState::TOOL_STATE_PING_OK;
    EXPECT_EQ(srv.response.state, res);

    // check id after added, -1 if failed or no tool connected
    EXPECT_TRUE(std::find(idList.begin(), idList.end(), srv.response.tool.id) != idList.end());

    // check motor type of tool, if unknow type and state is TOOL_STATE_PING_OK, that means no tool connected physically
    EXPECT_NE(static_cast<int>(srv.response.tool.motor_type), static_cast<int>(common::model::EHardwareType::UNKNOWN));
}

class ToolTestControlSuite : public ::testing::Test
{
  protected:
    static void SetUpTestCase() { pingTool(); }

    static void pingTool()
    {
        auto client = nh_g->serviceClient<tools_interface::PingDxlTool>("/niryo_robot/tools/ping_and_set_dxl_tool");

        bool exists(client.waitForExistence(ros::Duration(1)));
        EXPECT_TRUE(exists);

        tools_interface::PingDxlTool srv_ping;
        client.call(srv_ping);

        // Continue only if set tool successfully
        int res = common::model::ToolState::TOOL_STATE_PING_OK;
        ASSERT_EQ(srv_ping.response.state, res);

        id = srv_ping.response.tool.id;
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

    EXPECT_TRUE(std::find(id_list.begin(), id_list.end(), id) != id_list.end());
}

// open tool correct ID
TEST_F(ToolTestControlSuite, openTool)
{
    // only test tool if tool can be added and type gripper
    ASSERT_NE(id, -1);
    if (id != 11 && id != 12 && id != 13)
        return;

    tools_interface::ToolCommand srv;

    XmlRpc::XmlRpcValue filters;

    nh_g->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = static_cast<uint8_t>(id);
            srv.request.position = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["open_position"]));
            srv.request.speed = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["open_speed"]));
            srv.request.hold_torque = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["open_hold_torque"]));
            srv.request.max_torque = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["open_max_torque"]));
        }
    }

    auto client = nh_g->serviceClient<tools_interface::ToolCommand>("/niryo_robot/tools/open_gripper");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);
    client.call(srv);

    int res = common::model::ToolState::GRIPPER_STATE_OPEN;
    EXPECT_EQ(srv.response.state, res);
}

// open tool with wrong parameter
TEST_F(ToolTestControlSuite, openToolWrongId)
{
    tools_interface::ToolCommand srv;

    XmlRpc::XmlRpcValue filters;

    nh_g->getParam("tool_list", filters);

    // wrong id
    srv.request.id = static_cast<uint8_t>(20);
    srv.request.position = static_cast<uint16_t>(static_cast<int>(filters[1]["specs"]["open_position"]));
    srv.request.speed = static_cast<uint16_t>(static_cast<int>(filters[1]["specs"]["open_speed"]));
    srv.request.hold_torque = static_cast<int16_t>(static_cast<int>(filters[1]["specs"]["open_hold_torque"]));
    srv.request.max_torque = static_cast<int16_t>(static_cast<int>(filters[1]["specs"]["open_max_torque"]));

    auto client = nh_g->serviceClient<tools_interface::ToolCommand>("/niryo_robot/tools/open_gripper");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);
    client.call(srv);

    int res = common::model::ToolState::GRIPPER_STATE_OPEN;
    EXPECT_NE(srv.response.state, res);
}

TEST_F(ToolTestControlSuite, CloseTool)
{
    // only test tool if tool can be added and type gripper
    ASSERT_NE(id, -1);
    if (id != 11 && id != 12 && id != 13)
        return;

    tools_interface::ToolCommand srv;

    XmlRpc::XmlRpcValue filters;
    nh_g->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = static_cast<uint8_t>(id);
            srv.request.position = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["close_position"]));
            srv.request.speed = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["close_speed"]));
            srv.request.hold_torque = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["close_hold_torque"]));
            srv.request.max_torque = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["close_max_torque"]));
            break;
        }
    }

    auto client = nh_g->serviceClient<tools_interface::ToolCommand>("/niryo_robot/tools/close_gripper");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    client.call(srv);

    int res = common::model::ToolState::GRIPPER_STATE_CLOSE;
    EXPECT_EQ(srv.response.state, res);
}

// close tool with wrong parameter
TEST_F(ToolTestControlSuite, closeToolWrongId)
{
    tools_interface::ToolCommand srv;

    XmlRpc::XmlRpcValue filters;

    nh_g->getParam("tool_list", filters);

    // wrong id
    srv.request.id = static_cast<uint8_t>(20);
    srv.request.position = static_cast<uint16_t>(static_cast<int>(filters[1]["specs"]["close_position"]));
    srv.request.speed = static_cast<uint16_t>(static_cast<int>(filters[1]["specs"]["close_speed"]));
    srv.request.hold_torque = static_cast<int16_t>(static_cast<int>(filters[1]["specs"]["close_hold_torque"]));
    srv.request.max_torque = static_cast<int16_t>(static_cast<int>(filters[1]["specs"]["close_max_torque"]));

    auto client = nh_g->serviceClient<tools_interface::ToolCommand>("/niryo_robot/tools/close_gripper");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);
    client.call(srv);

    int res = common::model::ToolState::GRIPPER_STATE_CLOSE;
    EXPECT_NE(srv.response.state, res);
}

TEST_F(ToolTestControlSuite, PullAirVacuumPump)
{
    if (id != 31)
        return;
    tools_interface::ToolCommand srv;

    XmlRpc::XmlRpcValue filters;
    nh_g->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = static_cast<uint8_t>(id);
            srv.request.position = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["pull_air_position"]));
            srv.request.hold_torque = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["pull_air_hold_torque"]));
            srv.request.speed = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["pull_air_velocity"]));
            srv.request.max_torque = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["pull_air_max_torque"]));
            break;
        }
    }

    auto client = nh_g->serviceClient<tools_interface::ToolCommand>("/niryo_robot/tools/pull_air_vacuum_pump");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    client.call(srv);

    int res = common::model::ToolState::VACUUM_PUMP_STATE_PULLED;
    EXPECT_EQ(srv.response.state, res);
}

TEST_F(ToolTestControlSuite, PushAirVacuumPump)
{
    if (id != 31)
        return;
    tools_interface::ToolCommand srv;

    XmlRpc::XmlRpcValue filters;
    nh_g->getParam("tool_list", filters);

    for (int i = 0; i < filters.size(); i++)
    {
        if (static_cast<int>(filters[i]["id"]) == id)
        {
            srv.request.id = static_cast<uint8_t>(id);
            srv.request.position = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["push_air_position"]));
            srv.request.speed = static_cast<uint16_t>(static_cast<int>(filters[i]["specs"]["push_air_velocity"]));
            srv.request.max_torque = static_cast<int16_t>(static_cast<int>(filters[i]["specs"]["push_air_max_torque"]));
            break;
        }
    }

    auto client = nh_g->serviceClient<tools_interface::ToolCommand>("/niryo_robot/tools/push_air_vacuum_pump");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    client.call(srv);

    int res = common::model::ToolState::VACUUM_PUMP_STATE_PUSHED;
    EXPECT_EQ(srv.response.state, res);
}

TEST_F(ToolTestControlSuite, ToolReboot)
{
    ASSERT_NE(id, -1);

    auto client = nh_g->serviceClient<niryo_robot_msgs::Trigger>("/niryo_robot/tools/reboot");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::Trigger srv;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
    EXPECT_EQ(srv.response.message, "Tool reboot succeeded");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "tools_interface_service_client");

    nh_g = std::make_unique<ros::NodeHandle>("~");

    bool manual_tests{false};
    ros::NodeHandle nh_private("~");
    nh_private.getParam("manual_tests", manual_tests);
    ROS_DEBUG("manual_tests: %s", manual_tests ? "True" : "False");

    // remove manual tests if automatic mode
    if (!manual_tests)
        testing::GTEST_FLAG(filter) = "-ToolTestControlSuiteManual.*";

    return RUN_ALL_TESTS();
}
