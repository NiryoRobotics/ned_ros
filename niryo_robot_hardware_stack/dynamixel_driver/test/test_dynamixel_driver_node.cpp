#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <vector>

#include <dynamixel_driver/dxl_driver_core.hpp>

class DxlDriverTest {

    private:
        ros::NodeHandle nh;

        boost::shared_ptr<DynamixelDriver::DynamixelDriverCore> _dynamixel;

        std::vector<uint32_t> dxl_home_pose{2000, 2047 , 511};
        std::vector<uint32_t> dxl_pose_1{2047, 2047 , 511};
        std::vector<uint32_t> dxl_pose_2{2047, 2047 , 511};

        std::vector<uint8_t> id{2,3,6};

    public:
        DxlDriverTest()
        {
            ros::NodeHandle nodeHandle("~");

            _dynamixel.reset(new DynamixelDriver::DynamixelDriverCore());

            TestServiceActiveTorque();
            TestPublishPoseCmd();

            TestReceiveState();

        }

        void TestServiceActiveTorque()
        {
            ROS_INFO("active all arm motors");
            DynamixelDriver::SynchronizeMotorCmd cmd;
            cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_TORQUE);
            cmd.setMotorsId(id);
            cmd.setParams(std::vector<uint32_t> {true, true, true});
            _dynamixel->setDxlCommands(cmd);
            ros::Duration(1).sleep();
            cmd.setParams(std::vector<uint32_t> {false, false, false});
            _dynamixel->setDxlCommands(cmd);
            ros::Duration(1).sleep();

        }

        void TestPublishPoseCmd()
        {           
            ROS_INFO("move all arm motors");
            DynamixelDriver::SynchronizeMotorCmd cmd;

            cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_TORQUE);
            cmd.setMotorsId(id);
            cmd.setParams(std::vector<uint32_t> {true, true, true});
            ROS_INFO("Sending command 1");
            _dynamixel->setDxlCommands(cmd);
            ros::Duration(3).sleep();

            cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_POSITION);
            cmd.setParams(dxl_home_pose);
            ROS_INFO("Sending command 2");
            _dynamixel->setDxlCommands(cmd);
            ros::Duration(3).sleep();

            cmd.setParams(dxl_pose_1);
            ROS_INFO("Sending command 3");
            _dynamixel->setDxlCommands(cmd);
            ros::Duration(3).sleep();

            cmd.setParams(dxl_pose_2);
            ROS_INFO("Sending command 4");
            _dynamixel->setDxlCommands(cmd);
            ros::Duration(3).sleep();

            cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_TORQUE);
            cmd.setParams(std::vector<uint32_t> {false, false, false});
            ROS_INFO("Sending command 5");
            _dynamixel->setDxlCommands(cmd);
        }

        void TestReceiveState()
        {
            ros::Duration r(0.1);
            std::vector<DynamixelDriver::DxlMotorState> motor_state;
            for(int i = 0 ; i < 100 ; i++)
            {
                motor_state = _dynamixel->getDxlStates();
                for(int j = 0 ; j < motor_state.size() ; j++)
                {
                    std::cout << " " << motor_state.at(j).getId() << "    " << motor_state.at(j).getPositionState() << std::endl;
                }
                std::cout << " "  << std::endl;
                r.sleep();
            }
        }

};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "dynamixel_driver_test");
  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;
   
    DxlDriverTest test; 

    ros::waitForShutdown();
    
    ROS_INFO("shutdown node");
}