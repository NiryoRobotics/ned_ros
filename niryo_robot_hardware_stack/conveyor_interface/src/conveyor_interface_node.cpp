#include <ros/ros.h>
#include "conveyor_interface/conveyor_interface_core.hpp"
#include "stepper_driver/stepper_driver_core.hpp"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "conveyor_interface_node");
  
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;
   
    boost::shared_ptr<StepperDriver::StepperDriverCore> stepper(new StepperDriver::StepperDriverCore());
    ros::Duration(1).sleep();
    boost::shared_ptr<ConveyorInterfaceCore> conveyor_interface(new ConveyorInterfaceCore(stepper));
    ros::Duration(1).sleep();
    
    ros::spin();

    return 0;
}
