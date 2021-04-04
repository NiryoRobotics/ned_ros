#ifndef DXL_ENUM_HPP
#define DXL_ENUM_HPP

#include <string>

namespace DynamixelDriver
{
    enum class DxlCommandType
    {
        CMD_TYPE_POSITION=1,
        CMD_TYPE_VELOCITY=2,
        CMD_TYPE_EFFORT=3,
        CMD_TYPE_TORQUE=4,
        CMD_TYPE_PING=5,
        CMD_TYPE_LEARNING_MODE=6,
        CMD_TYPE_UNKNOWN=100
    };

    struct DxlMotorType
    {
        enum class type
        {
            MOTOR_TYPE_XL430=2,
            MOTOR_TYPE_XL320=3,
            MOTOR_TYPE_XL330=4,
            MOTOR_TYPE_XC430=5,
            MOTOR_TYPE_UNKNOWN=100
        };

        static type fromString(std::string str_type)
        {
            if("xl430" == str_type)
               return type::MOTOR_TYPE_XL430;
            else if("xc430" == str_type)
               return type::MOTOR_TYPE_XC430;
            else if("xl320" == str_type)
               return type::MOTOR_TYPE_XL320;
            else if("xl330" == str_type)
               return type::MOTOR_TYPE_XL330;

            return type::MOTOR_TYPE_UNKNOWN;
        }

        static std::string toString(type t)
        {
            switch(t) {
                case type::MOTOR_TYPE_XL430:
                    return "xl430";
                case type::MOTOR_TYPE_XC430:
                    return "xc430";
                case type::MOTOR_TYPE_XL330:
                    return "xl330";
                case type::MOTOR_TYPE_XL320:
                    return "xl320";
                default:
                    return "unknown type (" + std::to_string((int)t) + ")";
                break;
            }
            return "";
        }

    };

    using DxlMotorType_t = DxlMotorType::type;

} // DynamixelDriver

#endif

