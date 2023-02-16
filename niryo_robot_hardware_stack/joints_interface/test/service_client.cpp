/*
    joints_interface_service_client.cpp
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

#include "joints_interface/joints_interface_core.hpp"

static std::unique_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, deactivateLearningMode)
{
    auto client = nh->serviceClient<niryo_robot_msgs::SetBool>("/niryo_robot/learning_mode/activate");

    bool exists(client.waitForExistence(ros::Duration(5)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::SetBool srv;
    srv.request.value = false;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, activateLearningMode)
{
    auto client = nh->serviceClient<niryo_robot_msgs::SetBool>("/niryo_robot/learning_mode/activate");

    bool exists(client.waitForExistence(ros::Duration(10)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::SetBool srv;
    srv.request.value = true;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, resetControllerServer)
{
    auto client = nh->serviceClient<niryo_robot_msgs::Trigger>("/niryo_robot/joints_interface/steppers_reset_controller");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::Trigger srv;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, requestNewCalibration)
{
    auto client = nh->serviceClient<niryo_robot_msgs::Trigger>("/niryo_robot/joints_interface/request_new_calibration");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::Trigger srv;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, calibrateMotor)
{
    auto client = nh->serviceClient<niryo_robot_msgs::SetInt>("/niryo_robot/joints_interface/calibrate_motors");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::SetInt srv;
    srv.request.value = 1;  // AUTO_CALIBRATION;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joints_interface_service_client");

    nh = std::make_unique<ros::NodeHandle>();

    testing::InitGoogleTest(&argc, argv);

    ros::Duration(10.0).sleep();
    return RUN_ALL_TESTS();
}
