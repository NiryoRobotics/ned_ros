/*
    conveyor_service_client.cpp
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

#include "conveyor_interface/conveyor_interface_core.hpp"

static std::unique_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, setConveyor)
{
    auto client = nh->serviceClient<conveyor_interface::SetConveyor>("/niryo_robot/conveyor/ping_and_set_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::SetConveyor srv;
    srv.request.id = 6;
    srv.request.cmd = conveyor_interface::SetConveyor::Request::ADD;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
    EXPECT_NE(srv.response.id, srv.request.id);
}

TEST(TESTSuite, controlConveyor1)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 6;
    srv.request.control_on = true;
    srv.request.speed = 75;
    srv.request.direction = 1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, controlConveyor2)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 6;
    srv.request.control_on = true;
    srv.request.speed = 75;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}


TEST(TESTSuite, controlConveyor3)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 6;
    srv.request.control_on = false;
    srv.request.speed = 75;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, controlConveyor4)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 7;
    srv.request.control_on = true;
    srv.request.speed = 75;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}


TEST(TESTSuite, controlConveyor5)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 7;
    srv.request.control_on = false;
    srv.request.speed = 75;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, removeConveyor)
{
    auto client = nh->serviceClient<conveyor_interface::SetConveyor>("/niryo_robot/conveyor/ping_and_set_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::SetConveyor srv;
    srv.request.id = 13;  // to be checked
    srv.request.cmd = conveyor_interface::SetConveyor::Request::REMOVE;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "conveyor_interface_service_client");

    nh = std::make_unique<ros::NodeHandle>();

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
