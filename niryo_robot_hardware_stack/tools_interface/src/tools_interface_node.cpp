#include <ros/ros.h>
#include "tools_interface/tools_interface_core.hpp"
#include "dynamixel_driver/dxl_driver_core.hpp"

using namespace ToolsInterface;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tools_interface_node");

    ROS_DEBUG("Launching tools_interface_node");

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle nh;

    std::shared_ptr<ToolsInterfaceCore> tool;

    std::shared_ptr<DynamixelDriver::DynamixelDriverCore> dynamixel(new DynamixelDriver::DynamixelDriverCore());
    ros::Duration(1).sleep();
    tool.reset(new ToolsInterfaceCore(dynamixel));
    ros::Duration(1).sleep();
    //testVacuumPump(nh);
    //    testGripper2(nh);
    ros::waitForShutdown();

    ROS_INFO("Tools Interface - Shutdown node");
}
