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

#include <memory>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/service_client.h>

#include "common/model/tool_state.hpp"

#include "end_effector_interface/end_effector_interface_core.hpp"

#include "ttl_driver/ttl_interface_core.hpp"

static std::unique_ptr<ros::NodeHandle> nh;

struct HandleMsgReturn
{
    static constexpr int max_failures = 200;
    HandleMsgReturn() {}

    static bool getCorrectMsg(const end_effector_interface::EEButtonStatusConstPtr &data, int value)
    {
        bool data_received = false;
        for (int i = 0; i < max_failures; i++)
        {
            if (data && data->action == value)
            {
                data_received = true;
                break;
            }
            else
            {
                ros::spinOnce();
                ros::WallDuration(0.01).sleep();
            }
        }
        return data_received;
    }

    template <class T> static bool publisherIsOn(const boost::shared_ptr<T const> &data)
    {
        bool data_received = false;
        for (int i = 0; i < max_failures; i++)
        {
            if (data)
            {
                data_received = true;
                break;
            }
            else
            {
                ros::spinOnce();
                ros::WallDuration(0.01).sleep();
            }
        }
        return data_received;
    }
};
// Test service set Digital IO
TEST(EndEffectorTestTSuite, serviceTest)
{
    auto client = nh->serviceClient<end_effector_interface::SetEEDigitalOut>("end_effector_interface/set_ee_io_state");

    // wait for node launched
    bool exists(client.waitForExistence(ros::Duration(10.0)));
    EXPECT_TRUE(exists);

    end_effector_interface::SetEEDigitalOut srv;
    client.call(srv);

    EXPECT_EQ(srv.response.state, true);
}

// check if status of button from driver is published to upper layer or not
// in real hw, we can't check because of manually press on button.
TEST(EndEffectorTestSuite, publisherTestButtonCustom)
{
    end_effector_interface::EEButtonStatusConstPtr data;
    ros::Subscriber sub = nh->subscribe<end_effector_interface::EEButtonStatus>("end_effector_interface/custom_button_status", 10,
                                                                                [&data](const end_effector_interface::EEButtonStatusConstPtr msg) { data = msg; });

    // wait a while to get data from topic
    bool res = HandleMsgReturn::publisherIsOn<end_effector_interface::EEButtonStatus>(data);

    ASSERT_EQ(sub.getNumPublishers(), 1U);

    ASSERT_TRUE(res) << "No data sent from publisher on topic end_effector_interface/custom_button_status";
}

TEST(EndEffectorTestSuite, publisherTestButtonFreeDriver)
{
    end_effector_interface::EEButtonStatusConstPtr data;
    ros::Subscriber sub = nh->subscribe<end_effector_interface::EEButtonStatus>("end_effector_interface/free_drive_button_status", 10,
                                                                                [&data](const end_effector_interface::EEButtonStatusConstPtr msg) { data = msg; });

    // wait a while to get data from topic
    bool res = HandleMsgReturn::publisherIsOn<end_effector_interface::EEButtonStatus>(data);

    ASSERT_EQ(sub.getNumPublishers(), 1U);

    ASSERT_TRUE(res) << "No data sent from publisher on topic end_effector_interface/free_drive_button_status";
}

TEST(EndEffectorTestSuite, publisherTestSavePosition)
{
    end_effector_interface::EEButtonStatusConstPtr data;
    ros::Subscriber sub = nh->subscribe<end_effector_interface::EEButtonStatus>("end_effector_interface/save_pos_button_status", 10,
                                                                                [&data](const end_effector_interface::EEButtonStatusConstPtr msg) { data = msg; });

    // wait a while to get data from topic
    bool res = HandleMsgReturn::publisherIsOn(data);

    ASSERT_EQ(sub.getNumPublishers(), 1U);

    ASSERT_TRUE(res) << "No data sent from publisher on topic end_effector_interface/save_pos_button_status";
}

TEST(EndEffectorTestSuite, publisherDigitalIO)
{
    end_effector_interface::EEIOStateConstPtr data;
    ros::Subscriber sub =
        nh->subscribe<end_effector_interface::EEIOState>("end_effector_interface/io_state", 10, [&data](const end_effector_interface::EEIOStateConstPtr msg) { data = msg; });

    // wait a while to get data from topic
    bool res = HandleMsgReturn::publisherIsOn<end_effector_interface::EEIOState>(data);

    ASSERT_EQ(sub.getNumPublishers(), 1U);

    ASSERT_TRUE(res) << "No data sent from publisher on topic end_effector_interface/io_state";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "end_effector_interface_service_client");

    nh = std::make_unique<ros::NodeHandle>("");

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
