/*
    ttl_driver_service_client.cpp
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

#include <ros/service_client.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "ros/duration.h"
#include "ttl_driver/ttl_interface_core.hpp"

#include <memory>
#include <string>

static std::unique_ptr<ros::NodeHandle> nh;
static bool simulation_mode;

TEST(TESTSuite, SetLeds)
{
    auto client = nh->serviceClient<niryo_robot_msgs::SetInt>("/niryo_robot/ttl_driver/set_dxl_leds");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::SetInt srv;
    srv.request.value = 2;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, WriteCustomValue)
{
    auto client = nh->serviceClient<ttl_driver::WriteCustomValue>("/niryo_robot/ttl_driver/send_custom_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::WriteCustomValue srv;

    srv.request.id = 2;
    srv.request.reg_address = 64;  // Torque enable for xl430
    srv.request.value = 1;
    srv.request.byte_number = 1;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, WriteCustomValueWrongParam)
{
    auto client = nh->serviceClient<ttl_driver::WriteCustomValue>("/niryo_robot/ttl_driver/send_custom_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::WriteCustomValue srv;

    srv.request.id = 50;
    srv.request.reg_address = 64;  // Torque enable for xl430
    srv.request.value = 1;
    srv.request.byte_number = 1;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::WRONG_MOTOR_TYPE);
}

TEST(TESTSuite, ReadCustomValue)
{
    auto client = nh->serviceClient<ttl_driver::ReadCustomValue>("/niryo_robot/ttl_driver/read_custom_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::ReadCustomValue srv;
    srv.request.id = 2;
    srv.request.reg_address = 64;
    srv.request.byte_number = 1;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    if (!simulation_mode)
    {
        EXPECT_EQ(srv.response.value, 1);
    }
}

TEST(TESTSuite, WritePIDValue)
{
    auto client = nh->serviceClient<ttl_driver::WritePIDValue>("/niryo_robot/ttl_driver/write_pid_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::WritePIDValue srv;

    srv.request.id = 2;
    srv.request.pos_p_gain = 3264;
    srv.request.pos_i_gain = 0;
    srv.request.pos_d_gain = 9520;
    srv.request.vel_p_gain = 0;
    srv.request.vel_i_gain = 0;
    srv.request.ff1_gain = 0;
    srv.request.ff2_gain = 400;
    srv.request.vel_profile = 100;
    srv.request.acc_profile = 200;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
    ros::Duration(1.0).sleep();
}

TEST(TESTSuite, ReadPIDValue)
{
    auto client = nh->serviceClient<ttl_driver::ReadPIDValue>("/niryo_robot/ttl_driver/read_pid_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::ReadPIDValue srv;
    srv.request.id = 2;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
    EXPECT_EQ(srv.response.pos_p_gain, 3264);
    EXPECT_EQ(srv.response.pos_i_gain, 0);
    EXPECT_EQ(srv.response.pos_d_gain, 9520);
    EXPECT_EQ(srv.response.vel_p_gain, 0);
    EXPECT_EQ(srv.response.vel_i_gain, 0);
    EXPECT_EQ(srv.response.ff1_gain, 0);
    EXPECT_EQ(srv.response.ff2_gain, 400);
}

TEST(TESTSuite, ReadPIDValueWrongParam)
{
    auto client = nh->serviceClient<ttl_driver::ReadPIDValue>("/niryo_robot/ttl_driver/read_pid_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::ReadPIDValue srv;

    srv.request.id = 20;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::FAILURE);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ttl_driver_service_client");

    testing::InitGoogleTest(&argc, argv);

    nh = std::make_unique<ros::NodeHandle>("~");
    nh->getParam("simulation_mode", simulation_mode);

    return RUN_ALL_TESTS();
}
