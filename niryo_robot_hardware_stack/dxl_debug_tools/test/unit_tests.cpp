/*
    unit_tests.cpp
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
#define PROTOCOL_VERSION 2.0

#ifdef __arm
    #define DEFAULT_PORT "/dev/serial0"
#elifdef __aarch64__
    #define DEFAULT_PORT "/dev/ttyAMA0"
#else
    #define DEFAULT_PORT ""
#endif


// Bring in my package's API, which is what I'm testing
#include "niryo_robot_debug/dxl_tools.h"

#include <ros/ros.h>

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, testInit)
{
    // Setup Dxl communication
    std::string serial_port = DEFAULT_PORT;
    std::shared_ptr<dynamixel::PortHandler> portHandler(dynamixel::PortHandler::getPortHandler(serial_port.c_str()));
    std::shared_ptr<dynamixel::PacketHandler> packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));

    robotDebug::DxlTools dxlTools(portHandler, packetHandler);

    //for default baudrate, ok
    ASSERT_NE(dxlTools.setupDxlBus(1000000), -1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
