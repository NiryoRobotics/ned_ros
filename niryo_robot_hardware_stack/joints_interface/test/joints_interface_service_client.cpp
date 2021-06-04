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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>

#include "joints_interface/joints_interface_core.hpp"

static auto nh = std::make_unique<ros::NodeHandle>();

TEST(TESTSuite, calibrateMotor)
{
    ros::ServiceClient client = nh->serviceClient<niryo_robot_msgs::SetInt>("/niryo_robot/joints_interface/calibrate_motors");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::SetInt srv;
    srv.request.value = 1; //AUTO_CALIBRATION;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}


TEST(TESTSuite, deactivateLearningMode)
{
    ros::ServiceClient client = nh->serviceClient<niryo_robot_msgs::SetBool>("niryo_robot/learning_mode/activate");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::SetBool srv;
    srv.request.value = false;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}


TEST(TESTSuite, activateLearningMode)
{
    ros::ServiceClient client = nh->serviceClient<niryo_robot_msgs::SetBool>("niryo_robot/learning_mode/activate");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::SetBool srv;
    srv.request.value = true;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, requestNewCalibration)
{
    ros::ServiceClient client = nh->serviceClient<niryo_robot_msgs::Trigger>("/niryo_robot/joints_interface/request_new_calibration");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::Trigger srv;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, resetControllerServer)
{
    ros::ServiceClient client = nh->serviceClient<niryo_robot_msgs::Trigger>("/niryo_robot/joints_interface/steppers_reset_controller");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    niryo_robot_msgs::Trigger srv;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joints_interface_service_client");

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
