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

#include "conveyor_interface/conveyor_interface_core.hpp"
#include <gtest/gtest.h>
#include <memory>
#include <ros/ros.h>
#include <ros/service_client.h>

#include <string>

static std::unique_ptr<ros::NodeHandle> nh;

// publish
#define MAX_FAILURES 1000
#define WAITING_TIME 2.0

/**
 * @brief wait_spin
 * @param t
 */
void wait_spin(double t)
{
    // wait for t seconds
    double now = ros::Time::now().toSec();
    while (ros::Time::now().toSec() - now < t)
    {
        ros::spinOnce();
        ros::WallDuration(0.1).sleep();
    }
}

/**
 * @brief waitForMessage
 * @param msg
 */
void waitForMessage(const conveyor_interface::ConveyorFeedbackArrayConstPtr &msg)
{
    size_t i = 0;
    while (ros::ok() && msg == nullptr && i < MAX_FAILURES)
    {
        ros::spinOnce();
        ros::WallDuration(0.1).sleep();
        ++i;
    }
    if (i == MAX_FAILURES)
    {
        ADD_FAILURE();
    }

    wait_spin(2.0);
}

/**
 * @brief The ConveyorInterfaceAutoTestSuite class
 */
class ConveyorInterfaceTestSuiteAuto : public ::testing::Test
{
  protected:
    static void SetUpTestCase()
    {
        ROS_INFO("SetupTestCase : Auto");
        _nh_private = std::make_unique<ros::NodeHandle>("~");
    }

    static void TearDownTestCase() { ROS_INFO("TearDownTestCase : Auto"); }

    static std::shared_ptr<ros::NodeHandle> _nh_private;
};

std::shared_ptr<ros::NodeHandle> ConveyorInterfaceTestSuiteAuto::_nh_private;

/**
 * @brief The ConveyorInterfaceManualTestSuite class
 */
class ConveyorInterfaceTestSuiteManual : public ::testing::Test
{
  protected:
    static void SetUpTestCase() { ROS_INFO("SetupTestCase : Manual"); }

    static void TearDownTestCase() { ROS_INFO("TearDownTestCase : Manual"); }
};

//************************
//    One conveyor (id 9)
// setConveyor : set a newly connected conveyor
// controlConveyor1 : change speed
// controlConveyor2 : change direction and speed
// controlConveyor3 : change direction and speed and set running at false

TEST_F(ConveyorInterfaceTestSuiteAuto, setConveyor)
{
    auto client = nh->serviceClient<conveyor_interface::SetConveyor>("/niryo_robot/conveyor/ping_and_set_conveyor");

    bool exists(client.waitForExistence(ros::Duration(5)));

    EXPECT_TRUE(exists);

    conveyor_interface::SetConveyor srv;
    srv.request.id = 8;
    srv.request.cmd = conveyor_interface::SetConveyor::Request::ADD;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
    EXPECT_NE(srv.response.id, srv.request.id);
}

TEST_F(ConveyorInterfaceTestSuiteAuto, controlConveyor1)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 9;
    srv.request.control_on = true;
    srv.request.speed = 75;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    // subscribe to feedback (latch so we will not have lots of messages, only the ones changing)

    conveyor_interface::ConveyorFeedbackArrayConstPtr pcl;
    auto subscriber = nh->subscribe<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 1,
                                                                               [&pcl](const conveyor_interface::ConveyorFeedbackArrayConstPtr &msg) { pcl = msg; });

    waitForMessage(pcl);

    EXPECT_EQ(subscriber.getNumPublishers(), 1U);

    size_t i = 0;
    while (ros::ok() && pcl->conveyors.empty() && i < MAX_FAILURES)
    {
        ros::spinOnce();
        ros::WallDuration(0.1).sleep();
        ++i;
    }
    if (i == MAX_FAILURES)
    {
        ADD_FAILURE();
    }

    auto conv_vec = pcl->conveyors;
    ASSERT_EQ(conv_vec.size(), 1U);

    EXPECT_EQ(conv_vec.at(0).conveyor_id, srv.request.id);
    EXPECT_EQ(conv_vec.at(0).running, srv.request.control_on);
    EXPECT_EQ(conv_vec.at(0).speed, srv.request.speed);
    EXPECT_EQ(conv_vec.at(0).direction, srv.request.direction);

    wait_spin(WAITING_TIME);
}

TEST_F(ConveyorInterfaceTestSuiteAuto, controlConveyor2)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 9;
    srv.request.control_on = true;
    srv.request.speed = 100;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    // subscribe to feedback

    conveyor_interface::ConveyorFeedbackArrayConstPtr pcl;
    auto subscriber = nh->subscribe<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 1,
                                                                               [&pcl](const conveyor_interface::ConveyorFeedbackArrayConstPtr &msg) { pcl = msg; });

    waitForMessage(pcl);

    EXPECT_EQ(subscriber.getNumPublishers(), 1U);
    wait_spin(2.0);

    auto conv_vec = pcl->conveyors;
    ASSERT_EQ(conv_vec.size(), 1U);

    EXPECT_EQ(conv_vec.at(0).conveyor_id, srv.request.id);
    EXPECT_EQ(conv_vec.at(0).running, srv.request.control_on);
    EXPECT_EQ(conv_vec.at(0).speed, srv.request.speed);
    EXPECT_EQ(conv_vec.at(0).direction, srv.request.direction);

    wait_spin(WAITING_TIME);
}

TEST_F(ConveyorInterfaceTestSuiteAuto, controlConveyor3)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 9;
    srv.request.control_on = false;
    srv.request.speed = 60;
    srv.request.direction = 1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    // subscribe to feedback

    conveyor_interface::ConveyorFeedbackArrayConstPtr pcl;
    auto subscriber = nh->subscribe<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 1,
                                                                               [&pcl](const conveyor_interface::ConveyorFeedbackArrayConstPtr &msg) { pcl = msg; });

    waitForMessage(pcl);

    EXPECT_EQ(subscriber.getNumPublishers(), 1U);
    wait_spin(2.0);

    auto conv_vec = pcl->conveyors;
    ASSERT_EQ(conv_vec.size(), 1U);

    EXPECT_EQ(conv_vec.at(0).conveyor_id, srv.request.id);
    EXPECT_EQ(conv_vec.at(0).running, srv.request.control_on);
    EXPECT_EQ(conv_vec.at(0).speed, 0);

    wait_spin(WAITING_TIME);
}

//************************
//    One conveyor
// controlConveyorWrongId : control wrong id

TEST_F(ConveyorInterfaceTestSuiteAuto, controlConveyorWrongId)
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

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::CONVEYOR_ID_INVALID);
}

//************************
//    One conveyor
// removeConveyor : remove previous conveyor
// duplicateRemovingConveyor : try to remove again
// RemoveConveyorWrongId : remove wrong id

TEST_F(ConveyorInterfaceTestSuiteAuto, removeConveyor)
{
    auto client = nh->serviceClient<conveyor_interface::SetConveyor>("/niryo_robot/conveyor/ping_and_set_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::SetConveyor srv;
    srv.request.id = 9;
    srv.request.cmd = conveyor_interface::SetConveyor::Request::REMOVE;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST_F(ConveyorInterfaceTestSuiteAuto, duplicateRemovingConveyor)
{
    auto client = nh->serviceClient<conveyor_interface::SetConveyor>("/niryo_robot/conveyor/ping_and_set_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::SetConveyor srv;
    srv.request.id = 9;
    srv.request.cmd = conveyor_interface::SetConveyor::Request::REMOVE;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND);
}

TEST_F(ConveyorInterfaceTestSuiteAuto, RemoveConveyorWrongId)
{
    auto client = nh->serviceClient<conveyor_interface::SetConveyor>("/niryo_robot/conveyor/ping_and_set_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::SetConveyor srv;
    srv.request.id = 7;
    srv.request.cmd = conveyor_interface::SetConveyor::Request::REMOVE;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND);
}

//************************
//    Two conveyor
// setTwoConveyor : set two conveyor, one after the other. WARNING : You need to wait for ROS_WARN message to manually connect a second conveyor
//    Some config hides ROS_WARN message. Use catkin_make or rostest --text to see them
// removeAndResetConveyor : remove wrong id
// controlTwoConveyors : control two conv at the same time (check that each conv has the correct params)
// removeFirstConveyor : to have only one conv remaining, the second one

TEST_F(ConveyorInterfaceTestSuiteManual, setTwoConveyor)
{
    auto client = nh->serviceClient<conveyor_interface::SetConveyor>("/niryo_robot/conveyor/ping_and_set_conveyor");

    bool exists(client.waitForExistence(ros::Duration(5)));
    EXPECT_TRUE(exists);

    // set first conveyor
    conveyor_interface::SetConveyor srv;
    srv.request.id = 8;
    srv.request.cmd = conveyor_interface::SetConveyor::Request::ADD;
    client.call(srv);

    uint8_t first_id = static_cast<uint8_t>(srv.response.id);
    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
    EXPECT_NE(first_id, srv.request.id);

    for (int i = 0; i < 10; ++i)
    {
        ROS_WARN("################### Wait for second conveyor to be manually connected: (%d s) ###################", i);
        ros::Duration(1).sleep();

        client.call(srv);
        if (niryo_robot_msgs::CommandStatus::SUCCESS == srv.response.status)
            break;
    }

    // set second conveyor
    uint8_t second_id = static_cast<uint8_t>(srv.response.id);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
    EXPECT_NE(first_id, second_id);

    // subscribe to feedback

    conveyor_interface::ConveyorFeedbackArrayConstPtr pcl;
    auto subscriber = nh->subscribe<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 1,
                                                                               [&pcl](const conveyor_interface::ConveyorFeedbackArrayConstPtr &msg) { pcl = msg; });

    waitForMessage(pcl);

    EXPECT_EQ(subscriber.getNumPublishers(), 1U);
    wait_spin(2.0);

    auto conv_vec = pcl->conveyors;
    ASSERT_EQ(conv_vec.size(), 2U);

    EXPECT_NE(conv_vec.at(0).conveyor_id, conv_vec.at(1).conveyor_id);

    wait_spin(WAITING_TIME);
}

TEST_F(ConveyorInterfaceTestSuiteManual, removeAndResetConveyor)
{
    auto client = nh->serviceClient<conveyor_interface::SetConveyor>("/niryo_robot/conveyor/ping_and_set_conveyor");

    // remove first conveyor
    conveyor_interface::SetConveyor srv_rm;
    srv_rm.request.id = 9;
    srv_rm.request.cmd = conveyor_interface::SetConveyor::Request::REMOVE;
    client.call(srv_rm);

    EXPECT_EQ(srv_rm.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
    wait_spin(WAITING_TIME);

    // reset first conveyor -> now the order of the conveyors should be inverted
    conveyor_interface::SetConveyor srv_set;
    srv_set.request.id = 8;
    srv_set.request.cmd = conveyor_interface::SetConveyor::Request::ADD;
    client.call(srv_set);

    EXPECT_EQ(srv_set.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
    EXPECT_EQ(srv_set.response.id, srv_rm.request.id);

    wait_spin(WAITING_TIME);

    // subscribe to feedback

    conveyor_interface::ConveyorFeedbackArrayConstPtr pcl;
    auto subscriber = nh->subscribe<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 1,
                                                                               [&pcl](const conveyor_interface::ConveyorFeedbackArrayConstPtr &msg) { pcl = msg; });

    waitForMessage(pcl);

    EXPECT_EQ(subscriber.getNumPublishers(), 1U);
    wait_spin(2.0);

    auto conv_vec = pcl->conveyors;
    ASSERT_EQ(conv_vec.size(), 2U);

    EXPECT_NE(conv_vec.at(0).conveyor_id, conv_vec.at(1).conveyor_id);
    EXPECT_EQ(conv_vec.at(1).conveyor_id, srv_rm.request.id);  // expect first id to be now second

    wait_spin(WAITING_TIME);
}

TEST_F(ConveyorInterfaceTestSuiteManual, controlBothConveyors)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv_1;
    srv_1.request.id = 9;
    srv_1.request.control_on = true;
    srv_1.request.speed = 40;
    srv_1.request.direction = -1;
    client.call(srv_1);
    EXPECT_EQ(srv_1.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    conveyor_interface::ControlConveyor srv_2;
    srv_2.request.id = 10;
    srv_2.request.control_on = true;
    srv_2.request.speed = 33;
    srv_2.request.direction = -1;
    client.call(srv_2);

    EXPECT_EQ(srv_2.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    // subscribe to feedbacks

    conveyor_interface::ConveyorFeedbackArrayConstPtr pcl;
    auto subscriber = nh->subscribe<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 1,
                                                                               [&pcl](const conveyor_interface::ConveyorFeedbackArrayConstPtr &msg) { pcl = msg; });

    waitForMessage(pcl);

    EXPECT_EQ(subscriber.getNumPublishers(), 1U);
    wait_spin(2.0);

    auto conv_vec = pcl->conveyors;
    ASSERT_EQ(conv_vec.size(), 2U);

    // we remove and reset first conveyor, we should now have an inverted situation
    EXPECT_EQ(conv_vec.at(1).conveyor_id, srv_1.request.id);
    EXPECT_EQ(conv_vec.at(1).running, srv_1.request.control_on);
    EXPECT_EQ(conv_vec.at(1).speed, srv_1.request.speed);
    EXPECT_EQ(conv_vec.at(1).direction, srv_1.request.direction);

    EXPECT_EQ(conv_vec.at(0).conveyor_id, srv_2.request.id);
    EXPECT_EQ(conv_vec.at(0).running, srv_2.request.control_on);
    EXPECT_EQ(conv_vec.at(0).speed, srv_2.request.speed);
    EXPECT_EQ(conv_vec.at(0).direction, srv_2.request.direction);

    wait_spin(WAITING_TIME);
}

TEST_F(ConveyorInterfaceTestSuiteManual, removeFirstConveyor)
{
    auto client = nh->serviceClient<conveyor_interface::SetConveyor>("/niryo_robot/conveyor/ping_and_set_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::SetConveyor srv;
    srv.request.id = 9;
    srv.request.cmd = conveyor_interface::SetConveyor::Request::REMOVE;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

//************************
//    One conveyor (id 10)
// controlConveyor1_bis : change speed
// controlConveyor2_bis : change direction and speed
// controlConveyor3_bis : change direction and speed and set running at false
// removeConveyor_bis : to have only one conv remaining, the second one

TEST_F(ConveyorInterfaceTestSuiteManual, controlConveyor1_bis)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 10;
    srv.request.control_on = true;
    srv.request.speed = 75;
    srv.request.direction = 1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    // subscribe to feedback

    conveyor_interface::ConveyorFeedbackArrayConstPtr pcl;
    auto subscriber = nh->subscribe<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 1,
                                                                               [&pcl](const conveyor_interface::ConveyorFeedbackArrayConstPtr &msg) { pcl = msg; });

    waitForMessage(pcl);

    EXPECT_EQ(subscriber.getNumPublishers(), 1U);
    wait_spin(2.0);

    auto conv_vec = pcl->conveyors;
    ASSERT_EQ(conv_vec.size(), 1U);

    EXPECT_EQ(conv_vec.at(0).conveyor_id, srv.request.id);
    EXPECT_EQ(conv_vec.at(0).running, srv.request.control_on);
    EXPECT_EQ(conv_vec.at(0).speed, srv.request.speed);
    EXPECT_EQ(conv_vec.at(0).direction, srv.request.direction);
    wait_spin(WAITING_TIME);
}

TEST_F(ConveyorInterfaceTestSuiteManual, controlConveyor2_bis)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 10;
    srv.request.control_on = true;
    srv.request.speed = 100;
    srv.request.direction = -1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    // subscribe to feedback

    conveyor_interface::ConveyorFeedbackArrayConstPtr pcl;
    auto subscriber = nh->subscribe<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 1,
                                                                               [&pcl](const conveyor_interface::ConveyorFeedbackArrayConstPtr &msg) { pcl = msg; });

    waitForMessage(pcl);

    EXPECT_EQ(subscriber.getNumPublishers(), 1U);
    wait_spin(2.0);

    auto conv_vec = pcl->conveyors;
    ASSERT_EQ(conv_vec.size(), 1U);

    EXPECT_EQ(conv_vec.at(0).conveyor_id, srv.request.id);
    EXPECT_EQ(conv_vec.at(0).running, srv.request.control_on);
    EXPECT_EQ(conv_vec.at(0).speed, srv.request.speed);
    EXPECT_EQ(conv_vec.at(0).direction, srv.request.direction);

    wait_spin(WAITING_TIME);
}

TEST_F(ConveyorInterfaceTestSuiteManual, controlConveyor3_bis)
{
    auto client = nh->serviceClient<conveyor_interface::ControlConveyor>("/niryo_robot/conveyor/control_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::ControlConveyor srv;
    srv.request.id = 10;
    srv.request.control_on = false;
    srv.request.speed = 62;
    srv.request.direction = 1;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    // subscribe to feedback

    conveyor_interface::ConveyorFeedbackArrayConstPtr pcl;
    auto subscriber = nh->subscribe<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 1,
                                                                               [&pcl](const conveyor_interface::ConveyorFeedbackArrayConstPtr &msg) { pcl = msg; });

    waitForMessage(pcl);

    EXPECT_EQ(subscriber.getNumPublishers(), 1U);
    wait_spin(2.0);

    auto conv_vec = pcl->conveyors;
    ASSERT_EQ(conv_vec.size(), 1U);

    EXPECT_EQ(conv_vec.at(0).conveyor_id, srv.request.id);
    EXPECT_EQ(conv_vec.at(0).running, srv.request.control_on);
    EXPECT_EQ(conv_vec.at(0).speed, 0);

    wait_spin(WAITING_TIME);
}

TEST_F(ConveyorInterfaceTestSuiteManual, removeConveyor_bis)
{
    auto client = nh->serviceClient<conveyor_interface::SetConveyor>("/niryo_robot/conveyor/ping_and_set_conveyor");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    conveyor_interface::SetConveyor srv;
    srv.request.id = 10;
    srv.request.cmd = conveyor_interface::SetConveyor::Request::REMOVE;
    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "conveyor_interface_service_client");

    nh = std::make_unique<ros::NodeHandle>();

    bool manual_tests{false};
    ros::NodeHandle nh_private("~");
    nh_private.getParam("manual_tests", manual_tests);
    ROS_DEBUG("manual_tests: %s", manual_tests ? "True" : "False");

    // remove manual tests if automatic mode
    if (!manual_tests)
        testing::GTEST_FLAG(filter) = "-ConveyorInterfaceTestSuiteManual.*";

    return RUN_ALL_TESTS();
}
