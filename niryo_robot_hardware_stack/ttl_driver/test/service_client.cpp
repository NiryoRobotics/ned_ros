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

#include "ros/duration.h"
#include "ttl_driver/ttl_interface_core.hpp"

#include <string>

static std::unique_ptr<ros::NodeHandle> nh;

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
    std::string hw_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hw_version);

    auto client = nh->serviceClient<ttl_driver::WriteCustomValue>("/niryo_robot/ttl_driver/send_custom_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::WriteCustomValue srv;

    if (hw_version == "ned" || hw_version == "one")
    {
        srv.request.id = 2;
    }
    else
    {
        srv.request.id = 5;
    }
    srv.request.reg_address = 64;  // Torque enable for xl430
    srv.request.value = 1;
    srv.request.byte_number = 1;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, ReadCustomValue)
{
    std::string hw_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hw_version);

    auto client = nh->serviceClient<ttl_driver::ReadCustomValue>("/niryo_robot/ttl_driver/read_custom_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::ReadCustomValue srv;
    if (hw_version == "ned" || hw_version == "one")
    {
        srv.request.id = 2;
    }
    else
    {
        srv.request.id =  5;
    }
    srv.request.reg_address = 64;
    srv.request.byte_number = 1;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, WritePIDValue)
{
    std::string hw_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hw_version);

    auto client = nh->serviceClient<ttl_driver::WriteCustomValue>("/niryo_robot/ttl_driver/write_pid_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::WritePIDValue srv;

    if (hw_version == "ned" || hw_version == "one")
    {
        srv.request.id = 2;
    }
    else
    {
        srv.request.id = 5;
    }

    srv.request.pos_p_gain = 1;
    srv.request.pos_i_gain = 2;
    srv.request.pos_d_gain = 3;
    srv.request.vel_p_gain = 4;
    srv.request.vel_i_gain = 5;
    srv.request.ff1_gain = 6;
    srv.request.ff2_gain = 7;
    srv.request.vel_profile = 8;
    srv.request.acc_profile = 9;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, ReadPIDValue)
{
    std::string hw_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hw_version);

    auto client = nh->serviceClient<ttl_driver::ReadPIDValue>("/niryo_robot/ttl_driver/read_pid_value");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::ReadPIDValue srv;
    if (hw_version == "ned" || hw_version == "one")
    {
        srv.request.id = 2;
    }
    else
    {
        srv.request.id =  5;
    }

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, ReadWritePIDValue)
{
    std::string hw_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hw_version);

    auto client_write = nh->serviceClient<ttl_driver::WriteCustomValue>("/niryo_robot/ttl_driver/write_pid_value");
    auto client_read = nh->serviceClient<ttl_driver::ReadPIDValue>("/niryo_robot/ttl_driver/read_pid_value");

    ttl_driver::WritePIDValue srv_write;
    ttl_driver::ReadPIDValue srv_read;

    if (hw_version == "ned" || hw_version == "one")
    {
        srv_write.request.id = 2;
        srv_read.request.id = 2;
    }
    else
    {
        srv_write.request.id = 5;
        srv_read.request.id =  5;
    }

    srv_write.request.pos_p_gain = 10;
    srv_write.request.pos_i_gain = 20;
    srv_write.request.pos_d_gain = 30;
    srv_write.request.vel_p_gain = 40;
    srv_write.request.vel_i_gain = 50;
    srv_write.request.ff1_gain = 60;
    srv_write.request.ff2_gain = 70;
    srv_write.request.vel_profile = 80;
    srv_write.request.acc_profile = 90;

    bool exists_write(client_write.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists_write);
    client_write.call(srv_write);

    EXPECT_EQ(srv_write.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    ros::Duration(1).sleep();

    bool exists_read(client_read.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists_read);
    client_read.call(srv_read);

    EXPECT_EQ(srv_read.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    // expect read values to be equal to the one we just wrote
    EXPECT_EQ(srv_read.response.pos_p_gain, srv_write.request.pos_p_gain);
    EXPECT_EQ(srv_read.response.pos_i_gain, srv_write.request.pos_i_gain);
    EXPECT_EQ(srv_read.response.pos_d_gain, srv_write.request.pos_d_gain);
    EXPECT_EQ(srv_read.response.vel_p_gain, srv_write.request.vel_p_gain);
    EXPECT_EQ(srv_read.response.vel_i_gain, srv_write.request.vel_i_gain);
    EXPECT_EQ(srv_read.response.ff1_gain, srv_write.request.ff1_gain);
    EXPECT_EQ(srv_read.response.ff2_gain, srv_write.request.ff2_gain);
}

TEST(TESTSuite, WriteVelocityProfile)
{
    std::string hw_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hw_version);

    auto client = nh->serviceClient<ttl_driver::WriteVelocityProfile>("/niryo_robot/ttl_driver/write_velocity_profile");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::WriteVelocityProfile srv;

    if (hw_version == "ned" || hw_version == "one")
    {
        return;  // no ttl steppers
    }
    else
    {
        srv.request.id = 2;
    }

    srv.request.v_start = 1;
    srv.request.a_1 = 2;
    srv.request.v_1 = 3;
    srv.request.a_max = 4;
    srv.request.v_max = 5;
    srv.request.d_max = 6;
    srv.request.d_1 = 7;
    srv.request.v_stop = 8;

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, ReadVelocityProfile)
{
    std::string hw_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hw_version);

    auto client = nh->serviceClient<ttl_driver::ReadVelocityProfile>("/niryo_robot/ttl_driver/read_velocity_profile");

    bool exists(client.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists);

    ttl_driver::ReadVelocityProfile srv;
    if (hw_version == "ned" || hw_version == "one")
    {
        return;  // no steppers ttl
    }
    else
    {
        srv.request.id =  2;
    }

    client.call(srv);

    EXPECT_EQ(srv.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);
}

TEST(TESTSuite, ReadWriteVelocityProfile)
{
    std::string hw_version;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("hardware_version", hw_version);

    if (hw_version == "ned" || hw_version == "one")
      return;

    auto client_write = nh->serviceClient<ttl_driver::WriteVelocityProfile>("/niryo_robot/ttl_driver/write_velocity_profile");
    auto client_read = nh->serviceClient<ttl_driver::ReadVelocityProfile>("/niryo_robot/ttl_driver/read_velocity_profile");

    ttl_driver::WriteVelocityProfile srv_write;
    ttl_driver::ReadVelocityProfile srv_read;

    srv_write.request.id = 2;
    srv_read.request.id =  2;

    srv_write.request.v_start = 10;
    srv_write.request.a_1 = 20;
    srv_write.request.v_1 = 30;
    srv_write.request.a_max = 40;
    srv_write.request.v_max = 50;
    srv_write.request.d_max = 60;
    srv_write.request.d_1 = 70;
    srv_write.request.v_stop = 80;

    bool exists_write(client_write.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists_write);
    client_write.call(srv_write);

    EXPECT_EQ(srv_write.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    ros::Duration(1).sleep();

    bool exists_read(client_read.waitForExistence(ros::Duration(1)));
    EXPECT_TRUE(exists_read);
    client_read.call(srv_read);

    EXPECT_EQ(srv_read.response.status, niryo_robot_msgs::CommandStatus::SUCCESS);

    // expect read values to be equal to the one we just wrote
    EXPECT_EQ(srv_read.response.v_start, srv_write.request.v_start);
    EXPECT_EQ(srv_read.response.a_1, srv_write.request.a_1);
    EXPECT_EQ(srv_read.response.v_1, srv_write.request.v_1);
    EXPECT_EQ(srv_read.response.a_max, srv_write.request.a_max);
    EXPECT_EQ(srv_read.response.v_max, srv_write.request.v_max);
    EXPECT_EQ(srv_read.response.d_max, srv_write.request.d_max);
    EXPECT_EQ(srv_read.response.d_1, srv_write.request.d_1);
    EXPECT_EQ(srv_read.response.v_stop, srv_write.request.v_stop);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ttl_driver_service_client");

    testing::InitGoogleTest(&argc, argv);

    nh = std::make_unique<ros::NodeHandle>();

    bool simulation_mode;
    ros::NodeHandle nh_private("~");
    nh_private.getParam("simulation_mode", simulation_mode);

    if (simulation_mode)
    {
        testing::GTEST_FLAG(filter) = "-TESTSuite.WriteCustomValue:TESTSuite.ReadCustomValue:";
    }

    return RUN_ALL_TESTS();
}
