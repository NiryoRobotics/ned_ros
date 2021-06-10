#include "dynamixel_driver/single_motor_cmd.hpp"

namespace DynamixelDriver
{
    SingleMotorCmd::SingleMotorCmd()
    {
    }

    SingleMotorCmd::SingleMotorCmd(DxlCommandType type, uint8_t motor_id, uint32_t param)
    {
        _type = type;
        _id = motor_id;
        _param = param;
    }

    DxlCommandType SingleMotorCmd::getType() const
    {
        return _type;
    }

    void SingleMotorCmd::setType(DxlCommandType type)
    {
        _type = type;
    }

    void SingleMotorCmd::setId(uint8_t id)
    {
        _id = id;
    }

    uint8_t SingleMotorCmd::getId() const
    {
        return _id;
    }

    void SingleMotorCmd::setParam(uint32_t param)
    {
        _param = param;
    }

    uint32_t SingleMotorCmd::getParam() const
    {
        return _param;
    }

} // namespace DynamixelDriver
