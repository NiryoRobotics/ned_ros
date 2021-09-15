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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "ttl_driver/ttl_interface_core.hpp"

#include <string>

static std::unique_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, setLeds)
{
    auto client = nh->serviceClient<niryo_robot_msgs::SetInt>("/niryo_robot/ttl_driver/set_dxl_leds");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::SetInt srv;
    srv.request.value = 2;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, sendCustomValue)
{
    std::string hw_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hw_version);

    auto client = nh->serviceClient<ttl_driver::SendCustomValue>("/niryo_robot/ttl_driver/send_custom_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::SendCustomValue srv;

    if (hw_version == "ned2")
    {
        srv.request.motor_type = 2;  // xl430
        srv.request.id = 5;
    }
    else
    {
        srv.request.motor_type = 2;  // xl430
        srv.request.id = 2;
    }
    srv.request.reg_address = 64;  // Torque enable for xl430
    srv.request.value = 1;
    srv.request.byte_number = 1;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, readCustomValue)
{
    std::string hw_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hw_version);

    auto client = nh->serviceClient<ttl_driver::ReadCustomValue>("/niryo_robot/ttl_driver/read_custom_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::ReadCustomValue srv;
    if (hw_version == "ned2")
    {
        srv.request.motor_type = 2;  // xl430
        srv.request.id = 5;
    }
    else
    {
        srv.request.motor_type = 2;  // xl430
        srv.request.id = 2;
    }
    srv.request.reg_address = 6;
    srv.request.byte_number = 1;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ttl_driver_service_client");

    testing::InitGoogleTest(&argc, argv);

    nh = std::make_unique<ros::NodeHandle>();

    std::string hardware_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hardware_version);
    if (hardware_version == "fake")
        testing::GTEST_FLAG(filter) = "-TESTSuite.sendCustomValue:TESTSuite.readCustomValue";

    return RUN_ALL_TESTS();
}
