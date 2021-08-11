/*
    end_effector_interface_service_client.cpp
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

#include "end_effector_interface/end_effector_interface_core.hpp"

#include "ttl_driver/ttl_driver_core.hpp"

static std::unique_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, pingTool)
{
    auto client = nh->serviceClient<end_effector_interface::PingDxlTool>("/niryo_robot/tools/ping_and_set_dxl_tool");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    end_effector_interface::PingDxlTool srv;
    client.call(srv);

    // surprisingly we must first create a variable to have it work
    int res = common::model::ToolState::TOOL_STATE_PING_OK;
    EXPECT_EQ(srv.response.state, res);
}

TEST(TESTSuite, openTool)
{
    auto client = nh->serviceClient<end_effector_interface::OpenGripper>("/niryo_robot/tools/open_gripper");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    end_effector_interface::OpenGripper srv;
    srv.request.open_max_torque = 1023;
    srv.request.open_speed = 600;
    srv.request.open_position = 200;
    srv.request.open_hold_torque = 400;

    client.call(srv);

    int res = common::model::ToolState::GRIPPER_STATE_OPEN;
    EXPECT_EQ(srv.response.state, res);
}

TEST(TESTSuite, CloseGripper)
{
    auto client = nh->serviceClient<end_effector_interface::CloseGripper>("/niryo_robot/tools/close_gripper");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    end_effector_interface::CloseGripper srv;
    srv.request.close_speed = 600;
    srv.request.close_position = 200;
    srv.request.close_hold_torque = 400;
    srv.request.close_max_torque = 400;

    client.call(srv);

    int res = common::model::ToolState::GRIPPER_STATE_CLOSE;
    EXPECT_EQ(srv.response.state, res);
}

TEST(TESTSuite, ToolReboot)
{
    auto client = nh->serviceClient<std_srvs::Trigger>("/niryo_robot/tools/reboot");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    std_srvs::Trigger srv;
    client.call(srv);

    EXPECT_EQ(srv.response.success, true);
}

TEST(TESTSuite, PullAirVacuumPump)
{
    auto client = nh->serviceClient<end_effector_interface::PullAirVacuumPump>("/niryo_robot/tools/pull_air_vacuum_pump");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    end_effector_interface::PullAirVacuumPump srv;
    srv.request.pull_air_velocity = 1023;
    srv.request.pull_air_position = 200;
    srv.request.pull_air_max_torque = 500;
    srv.request.pull_air_hold_torque = 100;
    client.call(srv);

    int res = common::model::ToolState::VACUUM_PUMP_STATE_PULLED;
    EXPECT_EQ(srv.response.state, res);
}

TEST(TESTSuite, PushAirVacuumPump)
{
    auto client = nh->serviceClient<end_effector_interface::PushAirVacuumPump>("/niryo_robot/tools/push_air_vacuum_pump");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    end_effector_interface::PushAirVacuumPump srv;
    srv.request.push_air_velocity = 1023;
    srv.request.push_air_position = 100;
    srv.request.push_air_max_torque = 64000;
    client.call(srv);

    int res = common::model::ToolState::VACUUM_PUMP_STATE_PUSHED;
    EXPECT_EQ(srv.response.state, res);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_interface_service_client");

    nh = std::make_unique<ros::NodeHandle>();

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
