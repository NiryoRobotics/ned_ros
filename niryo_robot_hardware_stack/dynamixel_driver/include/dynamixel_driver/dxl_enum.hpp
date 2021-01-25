#ifndef DXL_ENUM_HPP
#define DXL_ENUM_HPP

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
    };

    enum class DxlMotorType
    {
        MOTOR_TYPE_XL430=2,
        MOTOR_TYPE_XL320=3
    };
}

#endif
