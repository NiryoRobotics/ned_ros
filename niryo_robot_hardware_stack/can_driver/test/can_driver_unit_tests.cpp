/*
    can_driver_unit_tests.cpp
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
#include "can_driver/can_driver_core.hpp"
#include "can_driver/can_driver.hpp"

// Bring in gtest
#include <gtest/gtest.h>
#include <ros/console.h>

// Declare a test
TEST(CanDriverTestSuite, testInitDriver)
{
    can_driver::CanDriver stepper;
    EXPECT_TRUE(stepper.isConnectionOk());
}

// Declare a test
/*TEST(CanDriverTestSuite, testInitDriverCore)
{
    can_driver::CanDriverCore can_core;
    EXPECT_TRUE(can_core.isConnectionOk());
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "can_driver_unit_tests");

  ros::NodeHandle nh;

  if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
     ros::console::notifyLoggerLevelsChanged();

  return RUN_ALL_TESTS();
}
