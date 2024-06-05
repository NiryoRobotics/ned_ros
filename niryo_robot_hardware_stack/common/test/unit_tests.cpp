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

// Bring in my package's API, which is what I'm testing
#include "common/model/dxl_motor_state.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "common/model/stepper_motor_state.hpp"

#include <cmath>
#include <string>
#include <tuple>

// Bring in gtest
#include <gtest/gtest.h>

namespace
{

using ::common::model::EComponentType;
using ::common::model::EHardwareType;
using ::common::model::HardwareTypeEnum;

// For common features
// EHardwareType : type of the dxl motor
// double : offset

class DXLCommonTest : public testing::TestWithParam<std::tuple<EHardwareType, EComponentType, double>>
{
  protected:
    DXLCommonTest()
        : dxlState{std::get<0>(GetParam()), std::get<1>(GetParam()), 1}
    {
        std::cout << "Test for MotorType " << HardwareTypeEnum(std::get<0>(GetParam())).toString() << std::endl;

        dxlState.setOffsetPosition(std::get<2>(GetParam()));
        dxlState.setLimitPositionMin(-2.0);
        dxlState.setLimitPositionMax(2.0);

        // define precision according to smallest motor step in radian
        // we divide by two because we are looking for the closest integer
        precision = (dxlState.getTotalAngle() / dxlState.getTotalRangePosition()) / RADIAN_TO_DEGREE;
        precision /= 2;
    }

    common::model::DxlMotorState dxlState;
    double precision{};
};

// TODO(CC) find a better way to avoid redundancy of this setup method
// For identity testing
// double : rad_pos
class DXLIdentityRadTest : public testing::TestWithParam<std::tuple<EHardwareType, EComponentType, double, double>>
{
  protected:
    DXLIdentityRadTest()
        : dxlState{std::get<0>(GetParam()), std::get<1>(GetParam()), 1}
    {
        std::cout << "Test for MotorType " << HardwareTypeEnum(std::get<0>(GetParam())).toString() << std::endl;

        dxlState.setLimitPositionMax(2.0);
        dxlState.setLimitPositionMin(-2.0);
        dxlState.setOffsetPosition(std::get<2>(GetParam()));

        // define precision according to smallest motor step in radian
        // we divide by two because we are looking for the closest integer
        precision = (dxlState.getTotalAngle() / dxlState.getTotalRangePosition()) / RADIAN_TO_DEGREE;
        precision /= 2;
    }

    common::model::DxlMotorState dxlState;
    double precision{};
};

// int : motor_pos
class DXLIdentityMotorTest : public testing::TestWithParam<std::tuple<EHardwareType, EComponentType, double, int>>
{
  protected:
    DXLIdentityMotorTest()
        : dxlState{std::get<0>(GetParam()), std::get<1>(GetParam()), 1}
    {
        std::cout << "Test for MotorType " << HardwareTypeEnum(std::get<0>(GetParam())).toString() << std::endl;

        dxlState.setOffsetPosition(std::get<2>(GetParam()));
        dxlState.setLimitPositionMin(-2.0);
        dxlState.setLimitPositionMax(2.0);

        // define precision according to smallest motor step in radian
        // we divide by two because we are looking for the closest integer
        precision = (dxlState.getTotalAngle() / dxlState.getTotalRangePosition()) / RADIAN_TO_DEGREE;
        precision /= 2;
    }

    common::model::DxlMotorState dxlState;
    double precision{};
};

TEST_P(DXLCommonTest, validity) { ASSERT_TRUE(dxlState.isValid()); }

TEST_P(DXLCommonTest, zero)
{
    // check 0 (middle pos)
    EXPECT_NEAR(dxlState.to_rad_pos(0), dxlState.getOffsetPosition(), precision) << "to_motor_pos failed";

    EXPECT_NEAR(dxlState.to_motor_pos(dxlState.getOffsetPosition()), 0, precision) << "to_motor_pos failed";
}

TEST_P(DXLCommonTest, extremeLow)
{
    // check to_rad_pos extreme values
    EXPECT_NEAR(dxlState.to_rad_pos(0), dxlState.getOffsetPosition(), precision) << "to_motor_pos failed";
}

TEST_P(DXLCommonTest, extremeHigh)
{
    double totalAngle_rad = dxlState.getTotalAngle() / RADIAN_TO_DEGREE;

    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getTotalRangePosition()), totalAngle_rad + dxlState.getOffsetPosition(), precision) << "to_motor_pos failed";
}

TEST_P(DXLIdentityRadTest, identityFromRad)
{
    // check combinations is identity
    double test_rad = std::get<3>(GetParam());
    if (test_rad > dxlState.getLimitPositionMax())
        EXPECT_NEAR(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)), dxlState.getLimitPositionMax(), precision);
    else if (test_rad < dxlState.getLimitPositionMin())
        EXPECT_NEAR(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)), dxlState.getLimitPositionMin(), precision);
    else
        EXPECT_NEAR(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)), test_rad, precision) << "to_rad_pos o to_motor_pos is not identity";
}

TEST_P(DXLIdentityMotorTest, identityFromMotorPos)
{
    // check combinations is identity
    int test_pos = std::get<3>(GetParam());
    if (dxlState.to_rad_pos(test_pos) > dxlState.getLimitPositionMax() || dxlState.to_rad_pos(test_pos) < dxlState.getLimitPositionMin())
    {
        EXPECT_TRUE(true);
    }
    else
        EXPECT_EQ(dxlState.to_motor_pos(dxlState.to_rad_pos(test_pos)), test_pos) << "to_motor_pos o to_rad_pos is not identity";
}

INSTANTIATE_TEST_CASE_P(CommonTests, DXLCommonTest,
                        testing::Combine(testing::Values(EHardwareType::XL320, EHardwareType::XL330, EHardwareType::XL430, EHardwareType::XC430),
                                         testing::Values(EComponentType::JOINT, EComponentType::END_EFFECTOR, EComponentType::TOOL), testing::Range(-2.0, 2.0, 0.6)));

INSTANTIATE_TEST_CASE_P(IdentityRadTests, DXLIdentityRadTest,
                        testing::Combine(testing::Values(EHardwareType::XL320, EHardwareType::XL330, EHardwareType::XL430, EHardwareType::XC430),
                                         testing::Values(EComponentType::JOINT, EComponentType::END_EFFECTOR, EComponentType::TOOL), testing::Range(-2.0, 2.0, 0.6),
                                         testing::Range(-M_PI, M_PI, 0.29)));

INSTANTIATE_TEST_CASE_P(IdentityMotorTests, DXLIdentityMotorTest,
                        testing::Combine(testing::Values(EHardwareType::XL320, EHardwareType::XL330, EHardwareType::XL430, EHardwareType::XC430),
                                         testing::Values(EComponentType::JOINT, EComponentType::END_EFFECTOR, EComponentType::TOOL), testing::Range(-2.0, 2.0, 0.6),
                                         testing::Range(0, 4000, 600)));

/**
 * @param rad
 * @param gear_ratio
 * @param direction
 */
class StepperIdentityRadTest : public testing::TestWithParam<std::tuple<double, double, int8_t>>
{
  protected:
    StepperIdentityRadTest()
        : stepperState{EHardwareType::STEPPER, EComponentType::JOINT, common::model::EBusProtocol::CAN, 1}
    {
        stepperState.setGearRatio(std::get<1>(GetParam()));
        stepperState.setDirection(std::get<2>(GetParam()));
        stepperState.setLimitPositionMax(3.0);
        stepperState.setLimitPositionMin(-3.0);
    }

    common::model::StepperMotorState stepperState;
    double precision = 0.001;
};

TEST_P(StepperIdentityRadTest, identityFromRad)
{
    double pos_rad = std::get<0>(GetParam());
    double pos_last = stepperState.to_rad_pos(stepperState.to_motor_pos(pos_rad));
    if (pos_rad > stepperState.getLimitPositionMax())
        EXPECT_NEAR(pos_last, stepperState.getLimitPositionMax(), precision);
    else if (pos_rad < stepperState.getLimitPositionMin())
        EXPECT_NEAR(pos_last, stepperState.getLimitPositionMin(), precision);
    else
        EXPECT_NEAR(pos_last, pos_rad, precision) << "to_rad_pos o to_motor_pos is not identity";
}

INSTANTIATE_TEST_CASE_P(IdentityRadTest, StepperIdentityRadTest, testing::Combine(testing::Range(-M_PI, M_PI, M_PI / 4), testing::Range(5.0, 500.0, 50.0), testing::Values(-1, 1)));
/**
 * @param pos
 * @param gear_ratio
 * @param direction
 */
class StepperIdentityMotorTest : public testing::TestWithParam<std::tuple<int, double, int8_t>>
{
  protected:
    StepperIdentityMotorTest()
        : stepperState{EHardwareType::STEPPER, EComponentType::JOINT, common::model::EBusProtocol::CAN, 1}
    {
        stepperState.setGearRatio(std::get<1>(GetParam()));
        stepperState.setDirection(std::get<2>(GetParam()));
        stepperState.setLimitPositionMin(-3.0);
        stepperState.setLimitPositionMax(3.0);
    }

    common::model::StepperMotorState stepperState;
    double precision = 0.001;
};

TEST_P(StepperIdentityMotorTest, identityFromRad)
{
    int pos = std::get<0>(GetParam());
    if (stepperState.to_rad_pos(pos) > stepperState.getLimitPositionMax() || stepperState.to_rad_pos(pos) < stepperState.getLimitPositionMin())
        EXPECT_TRUE(true);
    else
        EXPECT_NEAR(stepperState.to_motor_pos(stepperState.to_rad_pos(pos)), pos, precision) << "to_motor_pos o to_rad_pos is not identity";
}

INSTANTIATE_TEST_CASE_P(IdentityMotorTest, StepperIdentityMotorTest, testing::Combine(testing::Range(0, 2000, 500), testing::Range(5.0, 500.0, 50.0), testing::Values(-1, 1)));

// Global Tests
TEST(CommonTestSuite, testDefaultInvalid)
{
    common::model::DxlMotorState dxlState;
    common::model::StepperMotorState stepperState;

    EXPECT_FALSE(dxlState.isValid());
    EXPECT_FALSE(stepperState.isValid());
}

TEST(CommonTestSuite, testCreationDxlCmd)
{
    // DxlSingleCmd valid param
    common::model::DxlSingleCmd cmd;
    ASSERT_FALSE(cmd.isValid());

    common::model::DxlSingleCmd dxlCmd(common::model::EDxlCommandType::CMD_TYPE_POSITION, 5, std::initializer_list<uint32_t>{5});
    ASSERT_TRUE(dxlCmd.isValid());
    ASSERT_TRUE(dxlCmd.isDxlCmd());

    ASSERT_EQ(dxlCmd.getParam(), static_cast<uint8_t>(5));
    ASSERT_EQ(dxlCmd.getId(), static_cast<uint8_t>(5));

    dxlCmd.reset();
    ASSERT_FALSE(dxlCmd.isValid());
    ASSERT_NE(dxlCmd.getId(), static_cast<uint8_t>(5));
}

TEST(CommonTestSuite, testCreationStepperTtlCmd)
{
    // DxlSingleCmd valid param
    common::model::StepperTtlSingleCmd cmd(common::model::EStepperCommandType::CMD_TYPE_POSITION, 1, std::initializer_list<uint32_t>{5});
    ASSERT_TRUE(cmd.isValid());
    ASSERT_TRUE(cmd.isStepperCmd());

    ASSERT_EQ(cmd.getParam(), static_cast<uint8_t>(5));
    ASSERT_EQ(cmd.getId(), static_cast<uint8_t>(1));

    cmd.reset();
    ASSERT_FALSE(cmd.isValid());
    ASSERT_NE(cmd.getId(), static_cast<uint8_t>(1));
    ASSERT_EQ(cmd.getParam(), static_cast<uint8_t>(5));
}

TEST(CommonTestSuite, testCreationStepperCmd)
{
    // DxlSingleCmd valid param
    common::model::StepperSingleCmd cmd(common::model::EStepperCommandType::CMD_TYPE_POSITION, 1, std::initializer_list<int32_t>{5});
    ASSERT_TRUE(cmd.isValid());
    ASSERT_TRUE(cmd.isStepperCmd());

    ASSERT_EQ(cmd.getParam(), static_cast<uint8_t>(5));
    ASSERT_EQ(cmd.getId(), static_cast<uint8_t>(1));

    cmd.reset();
    ASSERT_FALSE(cmd.isValid());
    ASSERT_NE(cmd.getId(), static_cast<uint8_t>(1));
    ASSERT_EQ(cmd.getParam(), static_cast<uint8_t>(5));
}
}  // namespace

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
