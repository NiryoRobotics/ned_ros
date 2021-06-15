/*
    can_driver_test.cpp
    Copyright (C) 2021 Niryo
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
#include <ros/service_client.h>
#include <gtest/gtest.h>

//niryo
#include "can_driver/can_driver_core.hpp"
#include "common/model/stepper_command_type_enum.hpp"


static std::unique_ptr<ros::NodeHandle> nh(new ros::NodeHandle);

class CanDriverTest {

    private:
        ros::NodeHandle nh;

        std::shared_ptr<CanDriver::CanDriverCore> _can_driver;

        std::vector<int32_t> home_pose{32, 512 , 2322};
        std::vector<int32_t> pose_1{-833, 3046, 3476};
        std::vector<uint8_t> id{1,2,3};
        std::vector<common::model::StepperMotorState> states;

    public:
        CanDriverTest()
        {
            
            ros::NodeHandle nodeHandle("~");
            states.emplace_back(common::model::StepperMotorState(1));
            states.emplace_back(common::model::StepperMotorState(2));
            states.emplace_back(common::model::StepperMotorState(3));

            _can_driver.reset(new CanDriver::CanDriverCore());

            ros::Duration(2).sleep();
            for(auto const& sState : states) {
                common::model::StepperMotorCmd cmd_torque(common::model::EStepperCommandType::CMD_TYPE_TORQUE, sState.getId(), {false});
                _can_driver->addCommandToQueue(cmd_torque);
            }

            ros::Duration(2).sleep();
            // TestServiceActiveTorque();
            // TestPublishPoseCmd();

            // TestReceiveState();
            GetState();
        }
        
        void TestServiceActiveTorque()
        {
            ROS_INFO("active all arm motors");
            for(auto const& sState : states) {
                common::model::StepperMotorCmd cmd(common::model::EStepperCommandType::CMD_TYPE_TORQUE, sState.getId(), {true});
                _can_driver->addCommandToQueue(cmd);
            }
            ros::Duration(5).sleep();

            for(auto const& sState : states) {
                common::model::StepperMotorCmd cmd(common::model::EStepperCommandType::CMD_TYPE_TORQUE, sState.getId(), {true});
                _can_driver->addCommandToQueue(cmd);
            }
            ros::Duration(5).sleep();

        }

        void TestPublishPoseCmd()
        {           
            ROS_INFO("move all arm motors");

            for(auto const& sState : states) {
                common::model::StepperMotorCmd cmd(common::model::EStepperCommandType::CMD_TYPE_TORQUE, sState.getId(), {true});
                _can_driver->addCommandToQueue(cmd);
            }

            ros::Duration(1).sleep();

            for(size_t j = 0; j < states.size(); ++j) {
                common::model::StepperMotorCmd cmd(common::model::EStepperCommandType::CMD_TYPE_POSITION, states.at(j).getId(), {home_pose.at(j)});
                _can_driver->addCommandToQueue(cmd);
            }
            ros::Duration(5).sleep();

            for(size_t j = 0; j < states.size(); ++j) {
                common::model::StepperMotorCmd cmd(common::model::EStepperCommandType::CMD_TYPE_POSITION, states.at(j).getId(), {pose_1.at(j)});
                _can_driver->addCommandToQueue(cmd);
            }
            ros::Duration(5).sleep();

            for(auto const& sState : states) {
                common::model::StepperMotorCmd cmd(common::model::EStepperCommandType::CMD_TYPE_TORQUE, sState.getId(), {false});
                _can_driver->addCommandToQueue(cmd);
            }
            ros::Duration(1).sleep();
        }

        void GetState()
        {
            int freq = 100;
            ros::Rate control_loop_rate = ros::Rate(freq);
            std::vector<common::model::StepperMotorState> stepper_motor_state;
            while(ros::ok())
            {

                for(int i = 0 ; i < 1000 ; i++)
                {
                    stepper_motor_state = _can_driver->getStepperStates();
                    std::cout << stepper_motor_state.at(0).getPositionState() << " "
                              << stepper_motor_state.at(1).getPositionState() << " "
                              << stepper_motor_state.at(2).getPositionState() << std::endl;
                    control_loop_rate.sleep();
                }
            }
        }

        void TestReceiveState()
        {
            common::model::StepperMotorCmd cmd_torque(common::model::EStepperCommandType::CMD_TYPE_TORQUE, 1, {true});
            _can_driver->addCommandToQueue(cmd_torque);
            ros::Duration(1).sleep();

            int freq = 100;
            ros::Rate control_loop_rate = ros::Rate(freq);
            // setTrajectoryControllerCommands(std::vector<int32_t> &cmd)
            std::vector<int32_t> cmd = {2620, 0, 0};
            common::model::SynchronizeStepperMotorCmd cmd_send(common::model::EStepperCommandType::CMD_TYPE_POSITION);
            double p = 1.5;
            std::vector<common::model::StepperMotorState> stepper_motor_states;
            stepper_motor_states = _can_driver->getStepperStates();
            int32_t error = cmd.at(0) - static_cast<int32_t>(stepper_motor_states.at(0).getPositionState());
            while (std::abs(error) > 20)
            {
                stepper_motor_states = _can_driver->getStepperStates();
                error = static_cast<int>(cmd.at(0) - static_cast<int32_t>(stepper_motor_states.at(0).getPositionState())*p);

                cmd_send.addMotorParam(stepper_motor_states.at(0).getId(), cmd.at(0) + error);
                cmd_send.addMotorParam(stepper_motor_states.at(1).getId(), static_cast<int32_t>(stepper_motor_states.at(1).getPositionState()));
                cmd_send.addMotorParam(stepper_motor_states.at(2).getId(), static_cast<int32_t>(stepper_motor_states.at(2).getPositionState()));

                _can_driver->setSyncCommand(cmd_send);
                control_loop_rate.sleep();
            }

            cmd_torque.setParams({false});
            _can_driver->addCommandToQueue(cmd_torque);
            ros::Duration(1).sleep();
        }

};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "can_driver_test");
  
    ROS_DEBUG("Launching can_driver_test");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;
   
    CanDriverTest test; 

    ros::waitForShutdown();
    
    ROS_INFO("shutdown node");
}
