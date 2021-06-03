/*
    conveyor_test.cpp
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

#include "conveyor_interface/conveyor_interface_core.hpp"
#include "can_driver/can_driver_core.hpp"

TEST(TESTSuite, setConveyor)
{
    ros::ServiceClient client = nh.serviceClient<conveyor_interface::SetConveyor>("niryo_robot/conveyor/ping_and_set_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::SetConveyor srv;
    srv.request.id = 6;
    srv.request.activate = true;
    client.call(srv);

    EXPECT_EQ(srv.response.status, CommandStatus::SUCCESS);
    EXPECT_NE(srv.response.id, srv.request.id);

}

TEST(TESTSuite, controlConveyor1)
{

    ros::ServiceClient client = nh.serviceClient<conveyor_interface::ControlConveyor>("niryo_robot/tools/control_conveyor");
    
    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    
    conveyor_interface::ControlConveyor srv;
    srv.request.id = 6;
    srv.request.control_on = true;
    srv.request.speed = 75;
    srv.request.direction = 1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, CommandStatus::SUCCESS);

}

TEST(TESTSuite, controlConveyor2)
{

    ros::ServiceClient client = nh.serviceClient<conveyor_interface::ControlConveyor>("niryo_robot/tools/control_conveyor");
    
    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    
    conveyor_interface::ControlConveyor srv;
    srv.request.id = 6;
    srv.request.control_on = true;
    srv.request.speed = 75;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, CommandStatus::SUCCESS);

}


TEST(TESTSuite, controlConveyor3)
{

    ros::ServiceClient client = nh.serviceClient<conveyor_interface::ControlConveyor>("niryo_robot/tools/control_conveyor");
    
    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    
    conveyor_interface::ControlConveyor srv;
    srv.request.id = 6;
    srv.request.control_on = false;
    srv.request.speed = 75;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, CommandStatus::SUCCESS);

}

TEST(TESTSuite, updateConveyor)
{
    ros::ServiceClient client = nh.serviceClient<conveyor_interface::UpdateConveyorId>("niryo_robot/tools/update_conveyor_id");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::UpdateConveyorId srv;
    srv.request.old_id = 6;
    srv.request.new_id = 7;
    client.call(srv);

    EXPECT_EQ(srv.response.status, CommandStatus::SUCCESS);
}


TEST(TESTSuite, controlConveyor4)
{

    ros::ServiceClient client = nh.serviceClient<conveyor_interface::ControlConveyor>("niryo_robot/tools/control_conveyor");
    
    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    
    conveyor_interface::ControlConveyor srv;
    srv.request.id = 7;
    srv.request.control_on = true;
    srv.request.speed = 75;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, CommandStatus::SUCCESS);

}


TEST(TESTSuite, controlConveyor5)
{

    ros::ServiceClient client = nh.serviceClient<conveyor_interface::ControlConveyor>("niryo_robot/tools/control_conveyor");
    
    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    
    conveyor_interface::ControlConveyor srv;
    srv.request.id = 7;
    srv.request.control_on = false;
    srv.request.speed = 75;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, CommandStatus::SUCCESS);

}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "tools_interface_service_client");
    
// shardptr ?    nh.reset(new ros::NodeHandle);

    ros::NodeHandle nh;
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
