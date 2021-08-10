/*
    ttl_driver_unit_tests.cpp
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

// Bring in my package's API, which is what I'm testing
#include "ttl_driver/ttl_driver_core.hpp"
#include "ttl_driver/ttl_driver.hpp"

// Bring in gtest
#include <gtest/gtest.h>
#include <ros/console.h>

// Declare a test
TEST(TtlDriverTestSuite, testInitDriver)
{
    ros::NodeHandle nh;
    ttl_driver::TtlInterfaceCore ttl_interface(nh);
    ttl_driver.scanAndCheck();

    EXPECT_TRUE(ttl_driver.isConnectionOk());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ttl_driver_unit_tests");

  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}