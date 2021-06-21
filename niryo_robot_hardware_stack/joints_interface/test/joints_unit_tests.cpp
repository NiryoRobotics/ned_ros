/*
    joints_unit_tests.cpp
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
#include "joints_interface/joints_interface_core.hpp"

#include <vector>

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(JointsInterfaceTestSuite, testInitJoints)
{
    auto ttl_driver_core = std::make_shared<ttl_driver::TtlDriverCore>();
    auto can_driver_core = std::make_shared<can_driver::CanDriverCore>();

    joints_interface::JointsInterfaceCore joints_core(ttl_driver_core, can_driver_core);

    std::vector<std::shared_ptr<common::model::JointState> > jStates = joints_core.getJointsState();

    // expect list to be filled
    ASSERT_FALSE(jStates.empty());

    // expect valid states
    for (auto const& jState : jStates)
    {
        EXPECT_TRUE(jState->isValid());
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
