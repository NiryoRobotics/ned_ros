#include "dynamixel_driver/single_motor_cmd.hpp"
#include <sstream>

using namespace std;

namespace DynamixelDriver
{
    SingleMotorCmd::SingleMotorCmd(DxlCommandType_t type,
                                   uint8_t motor_id,
                                   uint32_t param) :
        _type(type),
        _id(motor_id),
        _param(param)
    {}

    void SingleMotorCmd::setType(DxlCommandType_t type)
    {
        _type = type;
    }

    void SingleMotorCmd::setId(uint8_t id)
    {
        _id = id;
    }

    void SingleMotorCmd::setParam(uint32_t param)
    {
        _param = param;
    }

    string SingleMotorCmd::str() const
    {
        ostringstream ss;
        ss << "Single motor cmd - ";

        ss << DxlCommandType::toString(_type);

        ss << ": ";
        ss << "motor " << _id << ": " << _param;

        return ss.str();
    }

} // namespace DynamixelDriver
