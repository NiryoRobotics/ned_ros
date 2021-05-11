#include "tools_interface/tool_state.hpp"

ToolState::ToolState(uint8_t id, DynamixelDriver::DxlMotorType type)
    : _id(id), _type(type)
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

void ToolState::setPosition(double position)
{
    _position = position;
}

double ToolState::getPosition()
{
    return _position;
}