/*
    stepper_driver_test.cpp
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

#include <memory>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <vector>

//niryo
#include "stepper_driver/stepper_driver_core.hpp"
#include "model/stepper_command_type_enum.hpp"


class StepperDriverTest {

    private:
        ros::NodeHandle nh;

        std::shared_ptr<StepperDriver::StepperDriverCore> _stepper;

        std::vector<int32_t> home_pose{32, 512 , 2322};
        std::vector<int32_t> pose_1{-833, 3046, 3476};
        std::vector<uint8_t> id{1,2,3};

    public:
        StepperDriverTest()
        {
            
            ros::NodeHandle nodeHandle("~");

            _stepper.reset(new StepperDriver::StepperDriverCore());

            ros::Duration(2).sleep();
            common::model::StepperMotorCmd cmd_torque;
            cmd_torque.setType(common::model::EStepperCommandType::CMD_TYPE_TORQUE);
            cmd_torque.setMotorsId(id);
            cmd_torque.setParams(std::vector<int32_t> {false, false, false});
            _stepper->setStepperCommands(cmd_torque);
            ros::Duration(2).sleep();
            // TestServiceActiveTorque();
            // TestPublishPoseCmd();

            // TestReceiveState();
            GetState();
        }
        
        void TestServiceActiveTorque()
        {
            ROS_INFO("active all arm motors");
            common::model::StepperMotorCmd cmd;
            cmd.setType(common::model::EStepperCommandType::CMD_TYPE_TORQUE);
            cmd.setMotorsId(id);
            cmd.setParams(std::vector<int32_t> {true, true, true});
            _stepper->setStepperCommands(cmd);
            ros::Duration(5).sleep();
            cmd.setParams(std::vector<int32_t> {false, false, false});
            _stepper->setStepperCommands(cmd);
            ros::Duration(5).sleep();

        }

        void TestPublishPoseCmd()
        {           
            ROS_INFO("move all arm motors");
            common::model::StepperMotorCmd cmd;

            cmd.setType(common::model::EStepperCommandType::CMD_TYPE_TORQUE);
            cmd.setMotorsId(id);
            cmd.setParams(std::vector<int32_t> {true, true, true});
            _stepper->setStepperCommands(cmd);
            ros::Duration(1).sleep();

            cmd.setType(common::model::EStepperCommandType::CMD_TYPE_POSITION);
            cmd.setParams(home_pose);
            _stepper->setStepperCommands(cmd);
            ros::Duration(5).sleep();

            cmd.setParams(pose_1);
            _stepper->setStepperCommands(cmd);
            ros::Duration(5).sleep();

            cmd.setType(common::model::EStepperCommandType::CMD_TYPE_TORQUE);
            cmd.setParams(std::vector<int32_t> {false, false, false});
            _stepper->setStepperCommands(cmd);
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
                    stepper_motor_state = _stepper->getStepperStates();
                    std::cout << stepper_motor_state.at(0).getPositionState() << " "
                              << stepper_motor_state.at(1).getPositionState() << " "
                              << stepper_motor_state.at(2).getPositionState() << std::endl;
                    control_loop_rate.sleep();
                }
            }
        }

        void TestReceiveState()
        {
            common::model::StepperMotorCmd cmd_torque;

            cmd_torque.setType(common::model::EStepperCommandType::CMD_TYPE_TORQUE);
            cmd_torque.setMotorsId(std::vector<uint8_t> {1});
            cmd_torque.setParams(std::vector<int32_t> {true});
            _stepper->setStepperCommands(cmd_torque);
            ros::Duration(1).sleep();

            int freq = 100;
            ros::Rate control_loop_rate = ros::Rate(freq);
            // setTrajectoryControllerCommands(std::vector<int32_t> &cmd)
            std::vector<int32_t> cmd = {2620, 0, 0};
            std::vector<int32_t> cmd_send = {0, 0, 0};
            double p = 1.5;
            std::vector<common::model::StepperMotorState> stepper_motor_state;
            stepper_motor_state = _stepper->getStepperStates();
            int32_t error = cmd.at(0) - static_cast<int32_t>(stepper_motor_state.at(0).getPositionState());
            while (std::abs(error) > 20)
            {
                stepper_motor_state = _stepper->getStepperStates();
                error = static_cast<int>(cmd.at(0) - static_cast<int32_t>(stepper_motor_state.at(0).getPositionState())*p);
                cmd_send.at(0) = cmd.at(0) + error;
                cmd_send.at(1) = static_cast<int32_t>(stepper_motor_state.at(1).getPositionState());
                cmd_send.at(2) = static_cast<int32_t>(stepper_motor_state.at(2).getPositionState());
                _stepper->setTrajectoryControllerCommands(cmd_send);
                control_loop_rate.sleep();
            }

            cmd_torque.setParams(std::vector<int32_t> {false});
            _stepper->setStepperCommands(cmd_torque);
            ros::Duration(1).sleep();
        }

};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "stepper_driver_test");
  
    ROS_DEBUG("Launching stepper_driver_test");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;
   
    StepperDriverTest test; 

    ros::waitForShutdown();
    
    ROS_INFO("shutdown node");
}
