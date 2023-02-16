/*
    hardware_interface_service_client.cpp
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

#include <gtest/gtest.h>
#include <memory>
#include <ros/ros.h>
#include <ros/service_client.h>

#include "niryo_robot_hardware_interface/hardware_interface.hpp"

static std::unique_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, launchMotorReport)
{
    auto client = nh->serviceClient<niryo_robot_msgs::Trigger>("/niryo_robot_hardware_interface/launch_motors_report");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::Trigger srv;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, stopMotorReport)
{
    // we need to launch a motor report before stopping it...
    auto client = nh->serviceClient<niryo_robot_msgs::Trigger>("/niryo_robot_hardware_interface/stop_motors_report");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::Trigger srv;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, rebootMotors)
{
    auto client = nh->serviceClient<niryo_robot_msgs::Trigger>("/niryo_robot_hardware_interface/reboot_motors");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::Trigger srv;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_interface_service_client");

    nh = std::make_unique<ros::NodeHandle>();

    ros::Duration(5.0).sleep();

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
