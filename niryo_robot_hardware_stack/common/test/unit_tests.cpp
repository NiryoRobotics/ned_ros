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

#include <string>
#include <math.h>

// Bring in gtest
#include <gtest/gtest.h>

namespace {


using ::common::model::EMotorType;
using ::common::model::MotorTypeEnum;


// For common features 
// EMotorType : type of the dxl motor
// double : offset

class DXLCommonTest : public testing::TestWithParam<std::tuple<EMotorType, double> >
{
    protected:
        void SetUp() override {
            std::cout << "Test for MotorType " << MotorTypeEnum(std::get<0>(GetParam())).toString() << std::endl;

            dxlState = common::model::DxlMotorState(std::get<0>(GetParam()),
                                                    common::model::EBusProtocol::TTL, 1);
            dxlState.setOffsetPosition(std::get<1>(GetParam()));

            //define precision according to smallest motor step in radian
            //we divide by two because we are looking for the closest integer
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
class DXLIdentityRadTest : public testing::TestWithParam<std::tuple<EMotorType, double, double> >
{
    protected:
    void SetUp() override {
        std::cout << "Test for MotorType " << MotorTypeEnum(std::get<0>(GetParam())).toString() << std::endl;

        dxlState = common::model::DxlMotorState(std::get<0>(GetParam()),
                                                common::model::EBusProtocol::TTL, 1);
        dxlState.setOffsetPosition(std::get<1>(GetParam()));

        //define precision according to smallest motor step in radian
        //we divide by two because we are looking for the closest integer
        ASSERT_NE(dxlState.getTotalRangePosition(), 0);
        precision = (dxlState.getTotalAngle() / dxlState.getTotalRangePosition())/RADIAN_TO_DEGREE;
        precision /= 2;
    }

        common::model::DxlMotorState dxlState;
        double precision;
};

// int : motor_pos
class DXLIdentityMotorTest : public testing::TestWithParam<std::tuple<EMotorType, double, int> >
{
    protected:
    void SetUp() override {
        std::cout << "Test for MotorType " << MotorTypeEnum(std::get<0>(GetParam())).toString() << std::endl;

        dxlState = common::model::DxlMotorState(std::get<0>(GetParam()),
                                                common::model::EBusProtocol::TTL, 1);
        dxlState.setOffsetPosition(std::get<1>(GetParam()));

        //define precision according to smallest motor step in radian
        //we divide by two because we are looking for the closest integer
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
                            testing::Values(EMotorType::XL320,
                                            EMotorType::XL330,
                                            EMotorType::XL430,
                                            EMotorType::XC430),
                            testing::Range(-2.0, 2.0, 0.6)));

INSTANTIATE_TEST_CASE_P(IdentityRadTests,
                         DXLIdentityRadTest,
                         testing::Combine(
                            testing::Values(EMotorType::XL320,
                                            EMotorType::XL330,
                                            EMotorType::XL430,
                                            EMotorType::XC430),
                            testing::Range(-2.0, 2.0, 0.6),
                            testing::Range(-M_PI, M_PI, 0.29)));

INSTANTIATE_TEST_CASE_P(IdentityMotorTests,
                         DXLIdentityMotorTest,
                         testing::Combine(
                            testing::Values(EMotorType::XL320,
                                            EMotorType::XL330,
                                            EMotorType::XL430,
                                            EMotorType::XC430),
                            testing::Range(-2.0, 2.0, 0.6),
                            testing::Range(0, 4000, 600)));
                            
// Global Tests
TEST(CommonTestSuite, testDefaultInvalid)
{
    common::model::DxlMotorState dxlState;
    common::model::StepperMotorState stepperState;

    EXPECT_FALSE(dxlState.isValid());
    EXPECT_FALSE(stepperState.isValid());
}

// Stepper Tests
TEST(CommonTestSuite, testStepper)
{
    common::model::StepperMotorState stepperState = common::model::StepperMotorState(common::model::EBusProtocol::CAN, 1);
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
    double precision = 0.0001;
    EXPECT_NEAR(stepperState.to_rad_pos(stepperState.to_motor_pos(test_rad)), test_rad, precision)
        << "to_rad_pos o to_motor_pos is not identity for motor ";

    EXPECT_EQ(stepperState.to_motor_pos(stepperState.to_rad_pos(test_pos)), test_pos)
                    << "to_motor_pos o to_rad_pos is not identity for motor ";
}

}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

