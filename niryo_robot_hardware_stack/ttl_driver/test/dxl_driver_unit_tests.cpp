/*
    dxl_driver_unit_tests.cpp
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

// Bring in my package's API, which is what I'm testing
#include "ttl_driver/dxl_driver_core.hpp"
#include "ttl_driver/dxl_driver.hpp"

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(DynamixelDriverTestSuite, testInitDriver)
{
    DynamixelDriver::DxlDriver dxl_driver;
    EXPECT_TRUE(dxl_driver.isConnectionOk());
}

// Declare a test
TEST(DynamixelDriverTestSuite, testInitDriverCore)
{
    DynamixelDriver::DxlDriverCore dxl_core;
    EXPECT_TRUE(dxl_core.isConnectionOk());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
