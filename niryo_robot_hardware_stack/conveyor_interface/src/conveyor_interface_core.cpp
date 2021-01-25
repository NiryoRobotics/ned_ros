#include "conveyor_interface/conveyor_interface_core.hpp"
#include "stepper_driver/conveyor_state.hpp"

ConveyorInterfaceCore::ConveyorInterfaceCore(boost::shared_ptr<StepperDriver::StepperDriverCore> &stepper):
   _stepper(stepper)
{
    _list_conveyor_id.clear();
    initParams();
    initServices();

    _conveyors_feedback_publisher = _nh.advertise<conveyor_interface::ConveyorFeedbackArray>("/niryo_robot/conveyor/feedback", 10);
    _publish_conveyors_feedback_thread.reset(new std::thread(boost::bind(&ConveyorInterfaceCore::_publishConveyorsFeedback, this)));
}

void ConveyorInterfaceCore::initParams()
{
    _nh.getParam("/niryo_robot_hardware_interface/conveyor/max_effort", _conveyor_max_effort);
    _nh.getParam("/niryo_robot_hardware_interface/conveyor/id", _conveyor_id);
    _nh.getParam("/niryo_robot_hardware_interface/conveyor/id_list", _list_possible_conveyor_id);
    _nh.getParam("/niryo_robot_hardware_interface/conveyor/publish_frequency", _publish_feedback_frequency);

    ROS_DEBUG("Conveyor interface - conveyor max effort : %d", _conveyor_max_effort);
    ROS_DEBUG("Conveyor interface - Publish_hw_status_frequency : %f", _publish_feedback_frequency);
    _list_available_id.push_back((uint8_t)_list_possible_conveyor_id.at(1));
    _list_available_id.push_back((uint8_t)_list_possible_conveyor_id.at(0));
}

void ConveyorInterfaceCore::initServices()
{
    _ping_and_set_stepper_server = _nh.advertiseService("/niryo_robot/conveyor/ping_and_set_conveyor", &ConveyorInterfaceCore::_callbackPingAndSetConveyor, this);
    _control_conveyor_server = _nh.advertiseService("/niryo_robot/conveyor/control_conveyor", &ConveyorInterfaceCore::_callbackControlConveyor, this);
}

bool ConveyorInterfaceCore::_callbackPingAndSetConveyor(conveyor_interface::SetConveyor::Request &req, conveyor_interface::SetConveyor::Response &res)
{
    std::string message = "";
    int result;
    if (_stepper->getCalibrationState() == true)
    {
        res.status = niryo_robot_msgs::CommandStatus::CALIBRATION_IN_PROGRESS;
        res.message = "Calibration in progress";
        return true;
    }

    uint8_t conveyor_id;
    if(req.cmd == conveyor_interface::SetConveyor::Request::ADD)
    {
        if(_list_available_id.size() == 0)
        {
            res.message = "no conveyor available";
            res.id = 0;
            res.status = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_LEFT;
            return true; 
        }
        else
        {
            conveyor_id = _list_available_id.back(); 
        }
        result = _stepper->setConveyor(conveyor_id);
        if(result != niryo_robot_msgs::CommandStatus::SUCCESS)
        {
            res.status = result;
            res.id = 0;
            ROS_INFO("Conveyor interface - No new conveyor found");
            message = "No new conveyor found";
        }
        else
        {
            _list_conveyor_id.push_back(conveyor_id);
            _list_available_id.pop_back();
            ros::Duration(0.05).sleep();

            StepperDriver::StepperMotorCmd cmd;
            std::vector<uint8_t> stepper_id{conveyor_id};
            std::vector<int32_t> stepper_params{8};
            cmd.setParams(stepper_params);
            cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_MICRO_STEPS);
            cmd.setMotorsId(stepper_id);
            _stepper->setStepperCommands(cmd);
            ros::Duration(0.05).sleep();

            stepper_params.clear();
            stepper_params = {_conveyor_max_effort};
            cmd.setParams(stepper_params);
            cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_MAX_EFFORT);
            cmd.setMotorsId(stepper_id);
            _stepper->setStepperCommands(cmd);
            ros::Duration(0.1).sleep();

            cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_CONVEYOR);
            cmd.setParams(std::vector<int32_t>{false, 0, -1});
            cmd.setMotorsId(stepper_id);
            _stepper->setConveyorCommands(cmd);
            ros::Duration(0.1).sleep();
            _stepper->setConveyorCommands(cmd);
            res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
            message = "Set new conveyor on id ";
            message += std::to_string(conveyor_id);
            message += " OK";
            res.id = conveyor_id;
        }
        
    }
    else if (req.cmd == conveyor_interface::SetConveyor::Request::REMOVE)
    {
        bool conveyor_found = false;
        for(int i = 0; i < _list_conveyor_id.size(); i++)
        {
            if(req.id == _list_conveyor_id.at(i))
            {
                conveyor_found = true;
                _list_conveyor_id.erase( _list_conveyor_id.begin() + i);
                _list_available_id.push_back(req.id);
                _stepper->unsetConveyor(req.id);
                std::sort(_list_available_id.begin(), _list_available_id.end(), std::greater<int>()); 
                message = "Remove conveyor id ";
                message += std::to_string(req.id);
                res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
                break;
            }
        }
        if(!conveyor_found)
        {
            ROS_INFO("Conveyor interface - Conveyor id %d not found", req.id);
            message = "Conveyor id ";
            message += std::to_string(req.id);
            message += " not found";
            res.status = niryo_robot_msgs::CommandStatus::NO_CONVEYOR_FOUND;
        }
    }
    res.message = message;
    return true;

}

bool ConveyorInterfaceCore::_callbackControlConveyor(conveyor_interface::ControlConveyor::Request &req, conveyor_interface::ControlConveyor::Response &res)
{
    std::string message = "";
    auto conveyor_id = std::find(_list_conveyor_id.begin(), _list_conveyor_id.end() , req.id);
    StepperDriver::StepperMotorCmd cmd;
    if(conveyor_id != _list_conveyor_id.end())
    {
        cmd.setMotorsId(std::vector<uint8_t> {req.id});
        cmd.setType(StepperDriver::StepperCommandType::CMD_TYPE_CONVEYOR);
        cmd.setParams(std::vector<int32_t>{req.control_on, req.speed, req.direction});
        message = "Set command on conveyor id ";
        message += std::to_string(req.id);
        message += " is OK";
        res.status = niryo_robot_msgs::CommandStatus::SUCCESS;
        _stepper->setConveyorCommands(cmd);
    }
    else
    {
        ROS_INFO("Conveyor interface - Conveyor id %d isn't set", req.id);
        message = "Conveyor id ";
        message += std::to_string(req.id);
        message += " is not set";
        res.status = niryo_robot_msgs::CommandStatus::CONVEYOR_ID_INVALID;
    }
    
    res.message = message;
    return true;
}

void ConveyorInterfaceCore::_publishConveyorsFeedback()
{
    ros::Rate publish_conveyor_feedback_rate = ros::Rate(_publish_feedback_frequency);

    while (ros::ok()) {
        
        conveyor_interface::ConveyorFeedbackArray msg;
        conveyor_interface::ConveyorFeedback data;

        std::vector<StepperDriver::ConveyorState> conveyor_list;
        conveyor_list = _stepper->getConveyorStates();
        for(int i = 0; i < conveyor_list.size(); i++)
        {
            data.conveyor_id = conveyor_list.at(i).getId();
            data.running = conveyor_list.at(i).getState();
            data.direction = conveyor_list.at(i).getDirection();
            data.speed = conveyor_list.at(i).getSpeed();
            msg.conveyors.push_back(data);
        }
        _conveyors_feedback_publisher.publish(msg);
        publish_conveyor_feedback_rate.sleep();

    }
}
