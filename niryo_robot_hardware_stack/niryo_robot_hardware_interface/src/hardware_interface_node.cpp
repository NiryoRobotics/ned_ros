#include <ros/ros.h>
#include "niryo_robot_hardware_interface/hardware_interface.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_interface_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::NodeHandle nh;

    NiryoRobotHardwareInterface::HardwareInterface nd(nh);
    ros::waitForShutdown();

    ROS_INFO("Hardware Interface - Shutdown node");
}
