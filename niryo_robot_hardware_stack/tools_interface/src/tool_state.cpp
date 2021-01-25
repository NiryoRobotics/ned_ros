#include "tools_interface/tool_state.hpp"

ToolState::ToolState(uint8_t id, std::string name, DynamixelDriver::DxlMotorType type)
    : _id(id), _name(name), _type(type)
{
    _connected = true;
    _position = 0.0; 
}

void ToolState::setId(uint8_t id)
{
    _id = id;
}

uint8_t ToolState::getId()
{
    return _id;
}

void ToolState::setType(DynamixelDriver::DxlMotorType type)
{
    _type = type;
}

DynamixelDriver::DxlMotorType ToolState::getType()
{
    return _type;
}

std::string& ToolState::getName()
{
    return _name;
}

void ToolState::setName(std::string name)
{
    _name = name;
}

void ToolState::setPosition(double position)
{
    _position = position;
}

double ToolState::getPosition()
{
    return _position;
}