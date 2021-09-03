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
#include "common/model/stepper_motor_state.hpp"
#include "common/model/single_motor_cmd.hpp"

#include <tuple>
#include <string>
#include <math.h>
#include <tuple>

// Bring in gtest
#include <gtest/gtest.h>

namespace {


using ::common::model::EHardwareType;
using ::common::model::HardwareTypeEnum;


// For common features
// EHardwareType : type of the dxl motor
// double : offset

class DXLCommonTest : public testing::TestWithParam<std::tuple<EHardwareType, double> >
{
    protected:
        void SetUp() override {
            std::cout << "Test for MotorType " << HardwareTypeEnum(std::get<0>(GetParam())).toString() << std::endl;

            dxlState = common::model::DxlMotorState(std::get<0>(GetParam()),
                                                    common::model::EBusProtocol::TTL, 1);
            dxlState.setOffsetPosition(std::get<1>(GetParam()));

            // define precision according to smallest motor step in radian
            // we divide by two because we are looking for the closest integer
            ASSERT_NE(dxlState.getTotalRangePosition(), 0);
            precision = (dxlState.getTotalAngle() / dxlState.getTotalRangePosition())/RADIAN_TO_DEGREE;
            precision /= 2;
        }

        common::model::DxlMotorState dxlState;
        double precision;
};

// TODO(CC) find a better way to avoid redundancy of this setup method
// For identity testing
// double : rad_pos
class DXLIdentityRadTest : public testing::TestWithParam<std::tuple<EHardwareType, double, double> >
{
    protected:
    void SetUp() override {
        std::cout << "Test for MotorType " << HardwareTypeEnum(std::get<0>(GetParam())).toString() << std::endl;

        dxlState = common::model::DxlMotorState(std::get<0>(GetParam()),
                                                common::model::EBusProtocol::TTL, 1);
        dxlState.setOffsetPosition(std::get<1>(GetParam()));

        // define precision according to smallest motor step in radian
        // we divide by two because we are looking for the closest integer
        ASSERT_NE(dxlState.getTotalRangePosition(), 0);
        precision = (dxlState.getTotalAngle() / dxlState.getTotalRangePosition())/RADIAN_TO_DEGREE;
        precision /= 2;
    }

        common::model::DxlMotorState dxlState;
        double precision;
};

// int : motor_pos
class DXLIdentityMotorTest : public testing::TestWithParam<std::tuple<EHardwareType, double, int> >
{
    protected:
    void SetUp() override {
        std::cout << "Test for MotorType " << HardwareTypeEnum(std::get<0>(GetParam())).toString() << std::endl;

        dxlState = common::model::DxlMotorState(std::get<0>(GetParam()),
                                                common::model::EBusProtocol::TTL, 1);
        dxlState.setOffsetPosition(std::get<1>(GetParam()));

        // define precision according to smallest motor step in radian
        // we divide by two because we are looking for the closest integer
        ASSERT_NE(dxlState.getTotalRangePosition(), 0);
        precision = (dxlState.getTotalAngle() / dxlState.getTotalRangePosition())/RADIAN_TO_DEGREE;
        precision /= 2;
    }

    common::model::DxlMotorState dxlState;
    double precision;
};

TEST_P(DXLCommonTest, validity) {
    ASSERT_TRUE(dxlState.isValid());
}

TEST_P(DXLCommonTest, zero) {
    // check 0 (middle pos)
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getMiddlePosition()),
                dxlState.getOffsetPosition(), precision) << "to_motor_pos failed";

    EXPECT_NEAR(dxlState.to_motor_pos(dxlState.getOffsetPosition()),
                dxlState.getMiddlePosition(), precision) << "to_motor_pos failed";
}

TEST_P(DXLCommonTest, extremeLow) {
    // check to_rad_pos extreme values
    double totalAngle_rad = dxlState.getTotalAngle() / RADIAN_TO_DEGREE;
    EXPECT_NEAR(dxlState.to_rad_pos(0), dxlState.getOffsetPosition() - totalAngle_rad/2, precision)
                 << "to_motor_pos failed";
}

TEST_P(DXLCommonTest, extremeHigh) {
    double totalAngle_rad = dxlState.getTotalAngle() / RADIAN_TO_DEGREE;

    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.getTotalRangePosition()),
                dxlState.getOffsetPosition() + totalAngle_rad/2, precision)
                << "to_motor_pos failed";
}

TEST_P(DXLIdentityRadTest, identityFromRad) {
    // check combinations is identity
    double test_rad = std::get<2>(GetParam());
    EXPECT_NEAR(dxlState.to_rad_pos(dxlState.to_motor_pos(test_rad)),
                test_rad, precision)
        << "to_rad_pos o to_motor_pos is not identity";
}

TEST_P(DXLIdentityMotorTest, identityFromMotorPos) {
    // check combinations is identity
    int test_pos = std::get<2>(GetParam());

    EXPECT_EQ(dxlState.to_motor_pos(dxlState.to_rad_pos(test_pos)), test_pos)
              << "to_motor_pos o to_rad_pos is not identity";
}

INSTANTIATE_TEST_CASE_P(CommonTests,
                         DXLCommonTest,
                         testing::Combine(
                            testing::Values(EHardwareType::XL320,
                                            EHardwareType::XL330,
                                            EHardwareType::XL430,
                                            EHardwareType::XC430),
                            testing::Range(-2.0, 2.0, 0.6)));

INSTANTIATE_TEST_CASE_P(IdentityRadTests,
                         DXLIdentityRadTest,
                         testing::Combine(
                            testing::Values(EHardwareType::XL320,
                                            EHardwareType::XL330,
                                            EHardwareType::XL430,
                                            EHardwareType::XC430),
                            testing::Range(-2.0, 2.0, 0.6),
                            testing::Range(-M_PI, M_PI, 0.29)));

INSTANTIATE_TEST_CASE_P(IdentityMotorTests,
                         DXLIdentityMotorTest,
                         testing::Combine(
                            testing::Values(EHardwareType::XL320,
                                            EHardwareType::XL330,
                                            EHardwareType::XL430,
                                            EHardwareType::XC430),
                            testing::Range(-2.0, 2.0, 0.6),
                            testing::Range(0, 4000, 600)));

/**
 * @param rad
 * @param gear_ratio
 * @param direction
*/
class StepperIdentityRadTest : public testing::TestWithParam<std::tuple<double, double, int> >
{
    protected:
        void SetUp() override
        {
            stepperState = common::model::StepperMotorState(common::model::EBusProtocol::CAN, 1);
            stepperState.setGearRatio(std::get<1>(GetParam()));
            stepperState.setDirection(std::get<2>(GetParam()));

            ASSERT_TRUE(stepperState.isValid());
            ASSERT_FALSE(stepperState.isConveyor());
            ASSERT_FALSE(stepperState.isDynamixel());
        }

        common::model::StepperMotorState stepperState;
        double precision = 0.0001;
};

TEST_P(StepperIdentityRadTest, identityFromRad)
{
    double pos_rad = std::get<0>(GetParam());
    EXPECT_NEAR(stepperState.to_rad_pos(stepperState.to_motor_pos(pos_rad)),
                pos_rad, precision) << "to_rad_pos o to_motor_pos is not identity";
}

INSTANTIATE_TEST_CASE_P(IdentityRadTest,
                        StepperIdentityRadTest,
                        testing::Combine(
                            testing::Range(-M_PI, M_PI, M_PI/4),
                            testing::Range(5.0, 500.0, 50.0),
                            testing::Values(-1, 1)));
/**
 * @param pos
 * @param gear_ratio
 * @param direction
*/
class StepperIdentityMotorTest : public testing::TestWithParam<std::tuple<int, double, int> >
{
    protected:
        void SetUp() override
        {
            stepperState = common::model::StepperMotorState(common::model::EBusProtocol::CAN, 1);
            stepperState.setGearRatio(std::get<1>(GetParam()));
            stepperState.setDirection(std::get<2>(GetParam()));

            ASSERT_TRUE(stepperState.isValid());
            ASSERT_FALSE(stepperState.isConveyor());
            ASSERT_FALSE(stepperState.isDynamixel());
        }

        common::model::StepperMotorState stepperState;
        double precision = 0.0001;
};

TEST_P(StepperIdentityMotorTest, identityFromRad)
{
    int pos = std::get<0>(GetParam());
    EXPECT_NEAR(stepperState.to_motor_pos(stepperState.to_rad_pos(pos)),
                pos, precision) << "to_motor_pos o to_rad_pos is not identity";
}

INSTANTIATE_TEST_CASE_P(IdentityMotorTest,
                        StepperIdentityMotorTest,
                        testing::Combine(
                            testing::Range(0, 2000, 500),
                            testing::Range(5.0, 500.0, 50.0),
                            testing::Values(-1, 1)));

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

    common::model::DxlSingleCmd dxlCmd(common::model::EDxlCommandType::CMD_TYPE_POSITION, 1, std::initializer_list<uint32_t>{5});
    ASSERT_TRUE(dxlCmd.isValid());
    ASSERT_TRUE(dxlCmd.isDxlCmd());

    ASSERT_EQ(dxlCmd.getParam(), static_cast<uint8_t>(5));
    ASSERT_EQ(dxlCmd.getId(), static_cast<uint8_t>(1));

    dxlCmd.reset();
    ASSERT_FALSE(dxlCmd.isValid());
    ASSERT_NE(dxlCmd.getId(), static_cast<uint8_t>(1));
    ASSERT_EQ(dxlCmd.getParam(), static_cast<uint8_t>(5));
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

