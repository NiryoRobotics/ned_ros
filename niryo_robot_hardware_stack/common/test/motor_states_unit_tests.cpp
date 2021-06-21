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
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#include <ros/ros.h>

// Bring in my package's API, which is what I'm testing
#include "common/model/dxl_motor_state.hpp"
#include "common/model/stepper_motor_state.hpp"

#include <string>

// Bring in gtest
#include <gtest/gtest.h>

static constexpr double precision = 0.002;

// Declare a test
TEST(CommonTestSuite, testDefaultInvalid)
{
    common::model::DxlMotorState dxlState;
    common::model::StepperMotorState stepperState;

    EXPECT_FALSE(dxlState.isValid());
    EXPECT_FALSE(stepperState.isValid());
}

TEST(CommonTestSuite, testXC430)
{
    common::model::DxlMotorState dxlState = DxlMotorState(EMotorType::XC430, 1);
    dxlState.setOffsetPosition(0.42);
    ASSERT_TRUE(dxlState.isValid());

    // check 0 (middle pos)
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getMiddlePosition()),
                dxlState.getOffsetPosition(), precision) << "to_motor_pos failed";

    EXPECT_NEAR(dxlState.to_motor_pos(dxlState.getOffsetPosition()),
                dxlState.getMiddlePosition(), precision) << "to_motor_pos failed";

    // check to_rad_pos extreme values
    double totalAngle_rad = dxlState.getTotalAngle() / RADIAN_TO_DEGREE;
    EXPECT_NEAR(dxlState.to_rad_pos(0), dxlState.getOffsetPosition() - totalAngle_rad/2,
                precision) << "to_motor_pos failed";

    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getTotalRangePosition()),
                dxlState.getOffsetPosition() + totalAngle_rad/2, precision)
                << "to_motor_pos failed";

    // check to_motor_pos extreme values
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition() + totalAngle_rad/2),
              static_cast<int>(dxlState.getTotalRangePosition())) << "to_motor_pos failed";

    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition() - totalAngle_rad/2), 0)
              << "to_motor_pos failed";

    // check combinations is identity
    double test_rad = M_PI/4;
    int test_pos = 500;
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)),
                test_rad, precision) << "to_rad_pos o to_motor_pos is not identity";

    EXPECT_EQ(dxlState.to_motor_pos(dxlState.to_rad_pos(test_pos)), test_pos)
              << "to_motor_pos o to_rad_pos is not identity";
}

TEST(CommonTestSuite, testXL430)
{
    common::model::DxlMotorState dxlState = DxlMotorState(EMotorType::XL430, 1);
    std::string type_str = MotorTypeEnum(dxlState.getType()).toString();
    dxlState.setOffsetPosition(0.42);
    ASSERT_TRUE(dxlState.isValid());

    // check 0 (middle pos)
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getMiddlePosition()),
                dxlState.getOffsetPosition(), precision) << "to_motor_pos failed";

    EXPECT_NEAR(dxlState.to_motor_pos(dxlState.getOffsetPosition()),
                dxlState.getMiddlePosition(), precision) << "to_motor_pos failed";

    // check to_rad_pos extreme values
    double totalAngle_rad = dxlState.getTotalAngle() / RADIAN_TO_DEGREE;
    EXPECT_NEAR(dxlState.to_rad_pos(0), dxlState.getOffsetPosition() - totalAngle_rad/2,
                precision) << "to_motor_pos failed";
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getTotalRangePosition()),
                dxlState.getOffsetPosition() + totalAngle_rad/2, precision) << "to_motor_pos failed";

    // check to_motor_pos extreme values
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition() + totalAngle_rad/2),
              static_cast<int>(dxlState.getTotalRangePosition())) << "to_motor_pos failed";
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition() - totalAngle_rad/2), 0)
                    << "to_motor_pos failed";

    // check combinations is identity
    double test_rad = M_PI/4;
    int test_pos = 1000;
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)), test_rad, precision)
                << "to_rad_pos o to_motor_pos is not identity";
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.to_rad_pos(test_pos)), test_pos)
              << "to_motor_pos o to_rad_pos is not identity";
}

TEST(CommonTestSuite, testXL330)
{
    common::model::DxlMotorState dxlState = common::model::DxlMotorState(common::model::EMotorType::XL330, 1);
    std::string type_str = common::model::MotorTypeEnum(dxlState.getType()).toString();
    dxlState.setOffsetPosition(0.42);
    ASSERT_TRUE(dxlState.isValid());

    // check 0 (middle pos)
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getMiddlePosition()),
                dxlState.getOffsetPosition(), precision) << "to_motor_pos failed";

    EXPECT_NEAR(dxlState.to_motor_pos(dxlState.getOffsetPosition()),
                dxlState.getMiddlePosition(), precision) << "to_motor_pos failed";

    // check to_rad_pos extreme values
    double totalAngle_rad = dxlState.getTotalAngle() / RADIAN_TO_DEGREE;
    EXPECT_NEAR(dxlState.to_rad_pos(0), dxlState.getOffsetPosition() - totalAngle_rad/2,
                precision) << "to_motor_pos failed";

    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getTotalRangePosition()),
                dxlState.getOffsetPosition() + totalAngle_rad/2, precision)
                << "to_motor_pos failed";

    // check to_motor_pos extreme values
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition() + totalAngle_rad/2),
              static_cast<int>(dxlState.getTotalRangePosition())) << "to_motor_pos failed";
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition() - totalAngle_rad/2), 0)
              << "to_motor_pos failed";

    // check combinations is identity
    double test_rad = M_PI/4;
    int test_pos = 1000;
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)), test_rad, precision)
                << "to_rad_pos o to_motor_pos is not identity";

    EXPECT_EQ(dxlState.to_motor_pos(dxlState.to_rad_pos(test_pos)), test_pos)
                << "to_motor_pos o to_rad_pos is not identity";
}

TEST(CommonTestSuite, testXL320)
{
    common::model::DxlMotorState dxlState = common::model::DxlMotorState(common::model::EMotorType::XL320, 1);
    std::string type_str = common::model::MotorTypeEnum(dxlState.getType()).toString();
    dxlState.setOffsetPosition(0.42);
    ASSERT_TRUE(dxlState.isValid());

    // check 0 (middle pos)
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getMiddlePosition()),
                dxlState.getOffsetPosition(), precision) << "to_motor_pos failed";

    EXPECT_NEAR(dxlState.to_motor_pos(dxlState.getOffsetPosition()),
                dxlState.getMiddlePosition(), precision) << "to_motor_pos failed";

    // check to_rad_pos extreme values
    double totalAngle_rad = dxlState.getTotalAngle() / RADIAN_TO_DEGREE;
    EXPECT_NEAR(dxlState.to_rad_pos(0),
                dxlState.getOffsetPosition() - totalAngle_rad/2, precision)
                << "to_motor_pos failed";

    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getTotalRangePosition()),
                dxlState.getOffsetPosition() + totalAngle_rad/2, precision)
            << "to_motor_pos failed";

    // check to_motor_pos extreme values
    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition() + totalAngle_rad/2),
              static_cast<int>(dxlState.getTotalRangePosition())) << "to_motor_pos failed";

    EXPECT_EQ(dxlState.to_motor_pos(dxlState.getOffsetPosition() - totalAngle_rad/2), 0)
              << "to_motor_pos failed";

    // check combinations is identity
    double test_rad = M_PI/4;
    int test_pos = 1000;
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)),
                test_rad, precision) << "to_rad_pos o to_motor_pos is not identity";

    EXPECT_EQ(dxlState.to_motor_pos(dxlState.to_rad_pos(test_pos)), test_pos)
              << "to_motor_pos o to_rad_pos is not identity";
}

TEST(CommonTestSuite, testStepper)
{
    common::model::StepperMotorState stepperState = common::model::StepperMotorState(1);
    stepperState.setGearRatio(800.0);
    stepperState.setDirection(-1);
    std::string type_str = common::model::MotorTypeEnum(stepperState.getType()).toString();
    ASSERT_TRUE(stepperState.isValid());

    // check to_rad_pos extreme values
    /*EXPECT_EQ(stepperState.to_rad_pos(0), stepperState.getOffsetPosition())
     * << "to_motor_pos failed for motor" << stepperState.str().c_str();
    EXPECT_EQ(stepperState.to_rad_pos(stepperState.getTotalRangePosition()),
        stepperState.getTotalAngle()) << "to_motor_pos failed for motor"
        << stepperState.str().c_str();

    // check to_motor_pos extreme values
    EXPECT_EQ(stepperState.to_motor_pos(stepperState.getOffsetPosition()),
              stepperState.getTotalRangePosition())
              << "to_motor_pos failed for motor"
              << stepperState.str().c_str();

    EXPECT_EQ(stepperState.to_motor_pos(stepperState.getTotalAngle()),
              static_cast<int>(dxlState.getTotalRangePosition()))
              << "to_motor_pos failed";
*/
    // check combinations is identity
    double test_rad = M_PI/3;
    int test_pos = 300;
    EXPECT_NEAR(stepperState.to_rad_pos(stepperState.to_motor_pos(test_rad)), test_rad, precision)
                    << "to_rad_pos o to_motor_pos is not identity for motor ";

    EXPECT_EQ(stepperState.to_motor_pos(stepperState.to_rad_pos(test_pos)), test_pos)
                    << "to_motor_pos o to_rad_pos is not identity for motor ";
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "common_test");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
