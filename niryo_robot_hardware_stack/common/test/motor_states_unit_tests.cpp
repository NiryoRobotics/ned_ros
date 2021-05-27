/*
    motor_states_unit_tests.cpp
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

// Bring in my package's API, which is what I'm testing
#include "model/dxl_motor_state.hpp"
#include "model/stepper_motor_state.hpp"


// Bring in gtest
#include <gtest/gtest.h>

using namespace common::model;

// Declare a test
TEST(TestSuite, testDefaultInvalid)
{
    DxlMotorState dxlState;
    StepperMotorState stepperState;

    EXPECT_FALSE(dxlState.isValid());
    EXPECT_FALSE(stepperState.isValid());

}

TEST(TestSuite, testXC430)
{
    DxlMotorState dxlState = DxlMotorState(EMotorType::XC430, 1);
    std::string type_str = MotorTypeEnum(dxlState.getType()).toString();
    ASSERT_TRUE(dxlState.isValid());

    //check to_rad_pos extreme values
    EXPECT_EQ(dxlState.to_rad_pos(0), dxlState.getOffsetPosition()) << "to_motor_pos failed for motor" << dxlState.str().c_str();
    EXPECT_EQ(dxlState.to_rad_pos(dxlState.getMiddlePosition()), 0) << "to_motor_pos failed for motor" << dxlState.str().c_str();

    // check to_motor_pos extreme values
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition()), 0) << "to_motor_pos failed for motor" << dxlState.str().c_str();
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getTotalAngle()), static_cast<int>(dxlState.getTotalRangePosition())) << "to_motor_pos failed for motor" << dxlState.str().c_str();

    //check combinations is identity
    double test_rad = M_PI/4;
    int test_pos = 1000;
    EXPECT_EQ(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)), test_rad) << "to_rad_pos o to_motor_pos is not identity for motor " << type_str.c_str();
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.to_rad_pos(test_pos)), test_pos) << "to_motor_pos o to_rad_pos is not identity for motor " << type_str.c_str();

}

TEST(TestSuite, testXL430)
{
    DxlMotorState dxlState = DxlMotorState(EMotorType::XL430, 1);
    std::string type_str = MotorTypeEnum(dxlState.getType()).toString();
    ASSERT_TRUE(dxlState.isValid());

    //check to_rad_pos extreme values
    EXPECT_EQ(dxlState.to_rad_pos(0), dxlState.getOffsetPosition()) << "to_motor_pos failed for motor" << dxlState.str().c_str();
    EXPECT_EQ(dxlState.to_rad_pos(dxlState.getMiddlePosition()), 0) << "to_motor_pos failed for motor" << dxlState.str().c_str();

    // check to_motor_pos extreme values
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition()), 0) << "to_motor_pos failed for motor" << dxlState.str().c_str();
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getTotalAngle()), static_cast<int>(dxlState.getTotalRangePosition())) << "to_motor_pos failed for motor" << dxlState.str().c_str();

    //check combinations is identity
    double test_rad = M_PI/4;
    int test_pos = 1000;
    EXPECT_EQ(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)), test_rad) << "to_rad_pos o to_motor_pos is not identity for motor " << type_str.c_str();
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.to_rad_pos(test_pos)), test_pos) << "to_motor_pos o to_rad_pos is not identity for motor " << type_str.c_str();

}

TEST(TestSuite, testXL330)
{
    DxlMotorState dxlState = DxlMotorState(EMotorType::XL330, 1);
    std::string type_str = MotorTypeEnum(dxlState.getType()).toString();
    ASSERT_TRUE(dxlState.isValid());

    //check to_rad_pos extreme values
    EXPECT_EQ(dxlState.to_rad_pos(0), dxlState.getOffsetPosition()) << "to_motor_pos failed for motor" << dxlState.str().c_str();
    EXPECT_EQ(dxlState.to_rad_pos(dxlState.getMiddlePosition()), 0) << "to_motor_pos failed for motor" << dxlState.str().c_str();

    // check to_motor_pos extreme values
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition()), 0) << "to_motor_pos failed for motor" << dxlState.str().c_str();
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getTotalAngle()), static_cast<int>(dxlState.getTotalRangePosition())) << "to_motor_pos failed for motor" << dxlState.str().c_str();

    //check combinations is identity
    double test_rad = M_PI/4;
    int test_pos = 1000;
    EXPECT_EQ(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)), test_rad) << "to_rad_pos o to_motor_pos is not identity for motor " << type_str.c_str();
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.to_rad_pos(test_pos)), test_pos) << "to_motor_pos o to_rad_pos is not identity for motor " << type_str.c_str();

}

TEST(TestSuite, testXL320)
{
    DxlMotorState dxlState = DxlMotorState(EMotorType::XL320, 1);
    std::string type_str = MotorTypeEnum(dxlState.getType()).toString();
    ASSERT_TRUE(dxlState.isValid());

    //check to_rad_pos extreme values
    EXPECT_EQ(dxlState.to_rad_pos(0), dxlState.getOffsetPosition()) << "to_motor_pos failed for motor" << dxlState.str().c_str();
    EXPECT_EQ(dxlState.to_rad_pos(dxlState.getMiddlePosition()), 0) << "to_motor_pos failed for motor" << dxlState.str().c_str();

    // check to_motor_pos extreme values
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition()), 0) << "to_motor_pos failed for motor" << dxlState.str().c_str();
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getTotalAngle()), static_cast<int>(dxlState.getTotalRangePosition())) << "to_motor_pos failed for motor" << dxlState.str().c_str();

    //check combinations is identity
    double test_rad = M_PI/4;
    int test_pos = 1000;
    EXPECT_EQ(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)), test_rad) << "to_rad_pos o to_motor_pos is not identity for motor " << type_str.c_str();
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.to_rad_pos(test_pos)), test_pos) << "to_motor_pos o to_rad_pos is not identity for motor " << type_str.c_str();

}

TEST(TestSuite, testStepper)
{
    StepperMotorState stepperState = StepperMotorState(1);
    std::string type_str = MotorTypeEnum(stepperState.getType()).toString();
    ASSERT_TRUE(stepperState.isValid());

    //check to_rad_pos extreme values
    /*EXPECT_EQ(stepperState.to_rad_pos(0), stepperState.getOffsetPosition()) << "to_motor_pos failed for motor" << stepperState.str().c_str();
    EXPECT_EQ(stepperState.to_rad_pos(stepperState.getTotalRangePosition()), stepperState.getTotalAngle()) << "to_motor_pos failed for motor" << stepperState.str().c_str();

    // check to_motor_pos extreme values
    EXPECT_EQ(stepperState.to_motor_pos(stepperState.getOffsetPosition()), stepperState.getTotalRangePosition()) << "to_motor_pos failed for motor" << stepperState.str().c_str();
    EXPECT_EQ(stepperState.to_motor_pos(stepperState.getTotalAngle()), static_cast<int>(dxlState.getTotalRangePosition())) << "to_motor_pos failed for motor" << dxlState.str().c_str();
*/
    //check combinations is identity
    double test_rad = M_PI/4;
    int test_pos = 1000;
    EXPECT_EQ(stepperState.to_rad_pos(stepperState.to_motor_pos(test_rad)), test_rad) << "to_rad_pos o to_motor_pos is not identity for motor " << type_str.c_str();
    EXPECT_EQ(stepperState.to_motor_pos(stepperState.to_rad_pos(test_pos)), test_pos) << "to_motor_pos o to_rad_pos is not identity for motor " << type_str.c_str();

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
