#include "tools_interface/tools_interface_core.hpp"

ToolsInterfaceCore::ToolsInterfaceCore(boost::shared_ptr<DynamixelDriver::DynamixelDriverCore> &dynamixel):
    _dynamixel(dynamixel)
{
    initParams();
    initServices();
    _tool.reset(new ToolState(0, DynamixelDriver::DxlMotorType::MOTOR_TYPE_XL320));
    _check_tool_connection_thread.reset(new std::thread(boost::bind(&ToolsInterfaceCore::_checkToolConnection, this)));

    pubToolId(0);
}

void ToolsInterfaceCore::initServices()
{
    _ping_and_set_dxl_tool_server = _nh.advertiseService("niryo_robot/tools/ping_and_set_dxl_tool", &ToolsInterfaceCore::_callbackPingAndSetDxlTool, this);
    _open_gripper_server = _nh.advertiseService("niryo_robot/tools/open_gripper", &ToolsInterfaceCore::_callbackOpenGripper, this);
    _close_gripper_server = _nh.advertiseService("niryo_robot/tools/close_gripper", &ToolsInterfaceCore::_callbackCloseGripper, this);
    _pull_air_vacuum_pump_server = _nh.advertiseService("niryo_robot/tools/pull_air_vacuum_pump", &ToolsInterfaceCore::_callbackPullAirVacuumPump, this);
    _push_air_vacuum_pump_server = _nh.advertiseService("niryo_robot/tools/push_air_vacuum_pump", &ToolsInterfaceCore::_callbackPushAirVacuumPump, this);
    _tool_reboot_server = _nh.advertiseService("niryo_robot/tools/reboot", &ToolsInterfaceCore::_callbackToolReboot, this);

    _current_tools_id_publisher = _nh.advertise<std_msgs::Int32>("/niryo_robot_hardware/tools/current_id", 1, true);
}

void ToolsInterfaceCore::initParams()
{
    std::vector<int> id_list;
    _tool_id_list.clear();
    _nh.getParam("/niryo_robot_hardware_interface/tools_interface/tools_params/id_list",id_list);
    _nh.getParam("/niryo_robot_hardware_interface/tools_interface/check_tool_connection_frequency",_check_tool_connection_frequency);

    std::string available_tools_list = "[";
    for(int i = 0 ; i < id_list.size() ; i++)
    {
        _tool_id_list.push_back(id_list[i]);
        if (i != 0) available_tools_list += ", ";
        available_tools_list += std::to_string(id_list[i]);
    }
    available_tools_list += "]";

    ROS_INFO("Tools Interface - List of tool ids : %s", available_tools_list.c_str());
}

void ToolsInterfaceCore::pubToolId(int id)
{
    std_msgs::Int32 msg;
    msg.data = id;
    _current_tools_id_publisher.publish(msg);
}

bool ToolsInterfaceCore::_callbackPingAndSetDxlTool(tools_interface::PingDxlTool::Request &req, tools_interface::PingDxlTool::Response &res)
{
    // A tool is already set
    if (_tool->getId() != 0)
    {
        res.id = _tool->getId();
        res.state = niryo_robot_msgs::CommandStatus::SUCCESS;
        return true;
    }

    std::lock_guard<std::mutex> lck(_tool_mutex);

    // Search for new tool
    std::vector<uint8_t> motor_list = _findToolMotorListWithRetries(5);

    bool tool_found = false;
    uint8_t tool_id;
    for(int i = 0; i < _tool_id_list.size(); i++)
    {
        std::vector<uint8_t>::iterator motor = std::find(motor_list.begin(), motor_list.end(), _tool_id_list[i]);
        if(motor != motor_list.end())
        {
            tool_id = _tool_id_list[i];
            tool_found = true;
            break;
        }
    }
    bool new_tool_set = false;
    if(tool_found)
    {
        new_tool_set = _equipToolWithRetries(tool_id, DynamixelDriver::DxlMotorType::MOTOR_TYPE_XL320, 5);
    }
    res.state = new_tool_set ? niryo_robot_msgs::CommandStatus::SUCCESS : TOOL_STATE_PING_OK;
    res.id = _tool->getId();
    pubToolId(_tool->getId());

    return true;
}

bool ToolsInterfaceCore::_callbackToolReboot(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (_tool->getId() != 0)
    {
        std::lock_guard<std::mutex> lck(_tool_mutex);
        res.success = _dynamixel->rebootMotor(_tool->getId(), _tool->getType());
        res.message = (res.success) ? "Tool reboot succeeded" : "Tool reboot failed";
        return true;
    }
    res.success = true;
    res.message = "No Tool";
    return true;
}

std::vector<uint8_t> ToolsInterfaceCore::_findToolMotorListWithRetries(unsigned int max_retries)
{
    std::vector<uint8_t> motor_list;
    while (max_retries > 0 && motor_list.empty())
    {
        motor_list = _dynamixel->scanTools();
        max_retries--;
        ros::Duration(0.05).sleep();
    }
    return motor_list;
}

bool ToolsInterfaceCore::_equipToolWithRetries(uint8_t tool_id, DynamixelDriver::DxlMotorType tool_type, unsigned max_retries)
{
    int result;
    while (max_retries-- > 0)
    {
        result = _dynamixel->setEndEffector(tool_id, tool_type);
        if (result == niryo_robot_msgs::CommandStatus::SUCCESS)
        {
            _tool.reset(new ToolState(tool_id, tool_type));
            _dynamixel->update_leds();
            break;
        }
        ros::Duration(0.05).sleep();
    }
    ROS_INFO("Tools Interface - Set End Effector return : %d", result);
    return result == niryo_robot_msgs::CommandStatus::SUCCESS;
}

bool ToolsInterfaceCore::_callbackOpenGripper(tools_interface::OpenGripper::Request &req, tools_interface::OpenGripper::Response &res)
{
    std::lock_guard<std::mutex> lck(_tool_mutex);
    if( req.id != _tool->getId() )
    {
        return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
    }
    DynamixelDriver::SingleMotorCmd cmd;
    std::vector<DynamixelDriver::SingleMotorCmd> list_cmd;
    cmd.setId(_tool->getId());

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_VELOCITY);
    cmd.setParam(req.open_speed);
    list_cmd.push_back(cmd);

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_POSITION);
    cmd.setParam(req.open_position);
    list_cmd.push_back(cmd);

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_EFFORT);
    cmd.setParam(1023);
    list_cmd.push_back(cmd);

    _dynamixel->setEndEffectorCommands(list_cmd);
    list_cmd.clear();

    double dxl_speed = (double)req.open_speed * (double)XL320_STEPS_FOR_1_SPEED; // position . sec-1
    double dxl_steps_to_do = abs((double)req.open_position - (double)_dynamixel->getEndEffectorState(_tool->getId(), _tool->getType())); // position
    double seconds_to_wait =  dxl_steps_to_do /  dxl_speed; // sec    
    ros::Duration(seconds_to_wait + 0.25).sleep();
    
    // set hold torque
    cmd.setId(_tool->getId());
    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_EFFORT);
    cmd.setParam(req.open_hold_torque);
    list_cmd.push_back(cmd);
    _dynamixel->setEndEffectorCommands(list_cmd);

    res.state = GRIPPER_STATE_OPEN;

    return true;    
}

bool ToolsInterfaceCore::_callbackCloseGripper(tools_interface::CloseGripper::Request &req, tools_interface::CloseGripper::Response &res)
{
    std::lock_guard<std::mutex> lck(_tool_mutex);
    if( req.id != _tool->getId() )
    {
        return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
    }

    DynamixelDriver::SingleMotorCmd cmd;
    std::vector<DynamixelDriver::SingleMotorCmd> list_cmd;
    cmd.setId(_tool->getId());
    
    int position_command = ( req.close_position < 50) ? 0 : req.close_position - 50;

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_VELOCITY);
    cmd.setParam(req.close_speed);
    list_cmd.push_back(cmd);

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_POSITION);
    cmd.setParam(position_command);
    list_cmd.push_back(cmd);

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_EFFORT);
    cmd.setParam(req.close_max_torque);
    list_cmd.push_back(cmd);

    _dynamixel->setEndEffectorCommands(list_cmd);
    list_cmd.clear();

    // calculate close duration
    double dxl_speed = (double)req.close_speed *(double) XL320_STEPS_FOR_1_SPEED; // position . sec-1
    double dxl_steps_to_do = abs((double)req.close_position - (double)_dynamixel->getEndEffectorState(_tool->getId(), _tool->getType())); // position
    double seconds_to_wait =  dxl_steps_to_do /  dxl_speed; // sec 
    ros::Duration(seconds_to_wait + 0.25).sleep();
   
    // set hold torque
    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_EFFORT);
    cmd.setParam(req.close_hold_torque);
    list_cmd.push_back(cmd);
    _dynamixel->setEndEffectorCommands(list_cmd);

    res.state = GRIPPER_STATE_CLOSE;

    return true; 
}

bool ToolsInterfaceCore::_callbackPullAirVacuumPump(tools_interface::PullAirVacuumPump::Request &req, tools_interface::PullAirVacuumPump::Response &res)
{
    std::lock_guard<std::mutex> lck(_tool_mutex);
    // check gripper id, in case no ping has been done before, or wrong id given
    if( req.id != _tool->getId() )
    {
        return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
    }
   
    int pull_air_velocity = 1023;

    // set vacuum pump pos, vel and torque
    DynamixelDriver::SingleMotorCmd cmd;
    std::vector<DynamixelDriver::SingleMotorCmd> list_cmd;
    cmd.setId(_tool->getId());

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_VELOCITY);
    cmd.setParam(pull_air_velocity);
    list_cmd.push_back(cmd);

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_POSITION);
    cmd.setParam(req.pull_air_position);
    list_cmd.push_back(cmd);

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_EFFORT);
    cmd.setParam(1023);
    list_cmd.push_back(cmd);

    _dynamixel->setEndEffectorCommands(list_cmd);
    list_cmd.clear();

    // calculate pull air duration
    double dxl_speed = (double)pull_air_velocity * (double)XL320_STEPS_FOR_1_SPEED; // position . sec-1
    double dxl_steps_to_do = abs((double)req.pull_air_position - (double)_dynamixel->getEndEffectorState(_tool->getId(), _tool->getType())); // position
    double seconds_to_wait = dxl_steps_to_do / dxl_speed; // sec

    ros::Duration(seconds_to_wait + 0.25).sleep();
    
    // set hold torque
    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_EFFORT);
    cmd.setParam(req.pull_air_hold_torque);
    list_cmd.push_back(cmd);
    _dynamixel->setEndEffectorCommands(list_cmd);

    res.state = VACUUM_PUMP_STATE_PULLED;

    return true;
}

bool ToolsInterfaceCore:: _callbackPushAirVacuumPump(tools_interface::PushAirVacuumPump::Request &req, tools_interface::PushAirVacuumPump::Response &res)
{
    std::lock_guard<std::mutex> lck(_tool_mutex);
    // check gripper id, in case no ping has been done before, or wrong id given
    if( req.id != _tool->getId() )
    {
        return niryo_robot_msgs::CommandStatus::TOOL_ID_INVALID;
    }

    int push_air_velocity = 1023;

    // set vacuum pump pos, vel and torque
    DynamixelDriver::SingleMotorCmd cmd;
    std::vector<DynamixelDriver::SingleMotorCmd> list_cmd;
    cmd.setId(_tool->getId());

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_VELOCITY);
    cmd.setParam(push_air_velocity);
    list_cmd.push_back(cmd);

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_POSITION);
    cmd.setParam(req.push_air_position);
    list_cmd.push_back(cmd);

    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_EFFORT);
    cmd.setParam(1023);
    list_cmd.push_back(cmd);

    _dynamixel->setEndEffectorCommands(list_cmd);
    list_cmd.clear();

    // calculate push air duration
    double dxl_speed = (double)push_air_velocity * (double)XL320_STEPS_FOR_1_SPEED; // position . sec-1
    double dxl_steps_to_do = abs((double)req.push_air_position - (double)_dynamixel->getEndEffectorState(_tool->getId(), _tool->getType())); // position
    double seconds_to_wait =  dxl_steps_to_do /  dxl_speed; // sec
    
    ros::Duration(seconds_to_wait + 0.25).sleep();

    // set torque to 0
    cmd.setType(DynamixelDriver::DxlCommandType::CMD_TYPE_EFFORT);
    cmd.setParam(0);
    list_cmd.push_back(cmd);
    _dynamixel->setEndEffectorCommands(list_cmd);

    res.state = VACUUM_PUMP_STATE_PUSHED;

    return true;    
}

void ToolsInterfaceCore::_checkToolConnection()
{
    ros::Rate check_connection = ros::Rate(_check_tool_connection_frequency);
    std_msgs::Int32 msg;
    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lck(_tool_mutex);
            std::vector<int> motor_list;
            motor_list = _dynamixel->getRemovedMotorList();
            for(int i = 0 ; i < motor_list.size(); i++)
            {
                if(_tool->getId() == motor_list.at(i))
                {
                    ROS_INFO("Tools Interface - Unset Current Tools");
                    _dynamixel->unsetEndEffector(_tool->getId(), _tool->getType());
                    _tool.reset(new ToolState(0, DynamixelDriver::DxlMotorType::MOTOR_TYPE_XL320));
                    msg.data = 0;
                    _current_tools_id_publisher.publish(msg);
                }
            }
        }
        check_connection.sleep();
    }
}
