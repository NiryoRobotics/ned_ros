#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <vector>

#include <stepper_driver/stepper_driver_core.hpp>

class StepperDriverTest {

    private:
        ros::NodeHandle nh;

        boost::shared_ptr<StepperDriver::StepperDriverCore> _stepper;

        std::vector<int32_t> home_pose{32, 512 , 2322};
        std::vector<int32_t> pose_1{-833, 3046, 3476};
        std::vector<uint8_t> id{1,2,3};

    public:
        StepperDriverTest()
        {
            
            ros::NodeHandle nodeHandle("~");

            _stepper.reset(new StepperDriver::StepperDriverCore());

            ros::Duration(2).sleep();
            StepperDriver::StepperMotorCmd cmd_torque;
            cmd_torque.setType(StepperDriver::StepperCommandType::CMD_TYPE_TORQUE);
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
            StepperDriver::StepperMotorCmd cmd;
            cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_TORQUE);
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
            StepperDriver::StepperMotorCmd cmd;

            cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_TORQUE);
            cmd.setMotorsId(id);
            cmd.setParams(std::vector<int32_t> {true, true, true});
            _stepper->setStepperCommands(cmd);
            ros::Duration(1).sleep();

            cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_POSITION);
            cmd.setParams(home_pose);
            _stepper->setStepperCommands(cmd);
            ros::Duration(5).sleep();

            cmd.setParams(pose_1);
            _stepper->setStepperCommands(cmd);
            ros::Duration(5).sleep();

            cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_TORQUE);
            cmd.setParams(std::vector<int32_t> {false, false, false});
            _stepper->setStepperCommands(cmd);
            ros::Duration(1).sleep();
        }

        void GetState()
        {
            int freq = 100;
            ros::Rate control_loop_rate = ros::Rate(freq);
            std::vector<int32_t> stepper_motor_state;
            while(ros::ok())
            {

                for(int i = 0 ; i < 1000 ; i++)
                {
                    stepper_motor_state = _stepper->getTrajectoryControllerStates();                
                    std::cout << stepper_motor_state[0] << " " << stepper_motor_state[1] << " " << stepper_motor_state[2] << std::endl;
                    control_loop_rate.sleep();
                }
            }
        }

        void TestReceiveState()
        {
            StepperDriver::StepperMotorCmd cmd_torque;

            cmd_torque.setType(StepperDriver::StepperCommandType::CMD_TYPE_TORQUE);
            cmd_torque.setMotorsId(std::vector<uint8_t> {1});
            cmd_torque.setParams(std::vector<int32_t> {true});
            _stepper->setStepperCommands(cmd_torque);
            ros::Duration(1).sleep();

            int freq = 100;
            ros::Rate control_loop_rate = ros::Rate(freq);
            // setTrajectoryControllerCommands(std::vector<int32_t> &cmd)
            std::vector<int32_t> cmd = {2620, 0, 0};
            std::vector<int32_t> cmd_send = {0, 0, 0};
            int p = 1.5;
            std::vector<int32_t> stepper_motor_state;
            stepper_motor_state = _stepper->getTrajectoryControllerStates();
            int32_t error = cmd[0] - stepper_motor_state[0];
            while (error > 20 || error < -20)
            {
                stepper_motor_state = _stepper->getTrajectoryControllerStates();
                error = (cmd[0] - stepper_motor_state[0])*p;
                cmd_send[0] = cmd[0] + error;
                cmd_send[1] = stepper_motor_state[1];
                cmd_send[2] = stepper_motor_state[2];
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
  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;
   
    StepperDriverTest test; 

    ros::waitForShutdown();
    
    ROS_INFO("shutdown node");
}