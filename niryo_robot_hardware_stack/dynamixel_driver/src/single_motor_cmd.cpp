#include "dynamixel_driver/single_motor_cmd.hpp"
#include <sstream>

namespace DynamixelDriver
{
    SingleMotorCmd::SingleMotorCmd(DxlCommandType type,
                                   uint8_t motor_id,
                                   uint32_t param) :
        _type(type),
        _id(motor_id),
        _param(param)
    {}

    void SingleMotorCmd::setType(DxlCommandType type)
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

    std::string SingleMotorCmd::str() const
    {
        std::ostringstream ss;
        ss << "Single motor cmd - ";

        switch(_type)
        {
            case DxlCommandType::CMD_TYPE_POSITION:
                ss << "Position";
                break;
            case DxlCommandType::CMD_TYPE_VELOCITY:
                ss << "Velocity";
                break;
            case DxlCommandType::CMD_TYPE_EFFORT:
                ss << "Effort";
                break;
            case DxlCommandType::CMD_TYPE_TORQUE:
                ss << "Torque";
                break;
            case DxlCommandType::CMD_TYPE_PING:
                ss << "Ping";
                break;
            case DxlCommandType::CMD_TYPE_LEARNING_MODE:
                ss << "Learning mode";
                break;
            default:
                ss << "Unknown type " << static_cast<int>(_type);
            break;
        }

        ss << ": ";
        ss << "motor " << _id << ": " << _param;

        return ss.str();
    }

} // namespace DynamixelDriver
