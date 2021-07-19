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

#include "ttl_driver/ttl_driver_core.hpp"

static std::unique_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, setLeds)
{
    ros::ServiceClient client = nh->serviceClient<niryo_robot_msgs::SetInt>("/niryo_robot/ttl_driver/set_dxl_leds");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::SetInt srv;
    srv.request.value = 2;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, sendCustomValue)
{
    ros::ServiceClient client = nh->serviceClient<ttl_driver::SendCustomDxlValue>("/niryo_robot/ttl_driver/send_custom_dxl_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::SendCustomDxlValue srv;
    /* to be defined
    srv.request.motor_type = ;
    srv.request.id = ;
    srv.request.reg_address = ;
    srv.request.value = ;
    srv.request.byte_number = ;

    client.call(srv);
*/
    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, readCustomValue)
{
    ros::ServiceClient client = nh->serviceClient<ttl_driver::ReadCustomDxlValue>("/niryo_robot/ttl_driver/read_custom_dxl_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::ReadCustomDxlValue srv;
    /* to be defined
    srv.request.motor_type = ;
    srv.request.id = ;
    srv.request.reg_address = ;
    srv.request.byte_number = ;

    client.call(srv);
*/
    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ttl_driver_service_client");

    testing::InitGoogleTest(&argc, argv);

    nh = std::make_unique<ros::NodeHandle>();

    return RUN_ALL_TESTS();
}
