#ifndef STEPPER_ENUM_HPP
#define STEPPER_ENUM_HPP

#include <string>

namespace StepperDriver
{
    enum class StepperCommandType
    {
        CMD_TYPE_NONE=0,
        CMD_TYPE_POSITION=1,
        CMD_TYPE_VELOCITY=2,
        CMD_TYPE_EFFORT=3,
        CMD_TYPE_TORQUE=4,
        CMD_TYPE_SYNCHRONIZE=5,
        CMD_TYPE_RELATIVE_MOVE=6,
        CMD_TYPE_MAX_EFFORT=7,
        CMD_TYPE_MICRO_STEPS=8,
        CMD_TYPE_POSITION_OFFSET=9,
        CMD_TYPE_CALIBRATION=10,
        CMD_TYPE_CONVEYOR=11,
        CMD_TYPE_UPDATE_CONVEYOR=12,
        CMD_TYPE_LEARNING_MODE=13,
    };

    struct StepperMotorType {
        enum class type
        {
            MOTOR_TYPE_STEPPER=1,
            MOTOR_TYPE_UNKNOWN=100
        };

        static type fromString(std::string str_type)
        {
            if("stepper" == str_type)
               return type::MOTOR_TYPE_STEPPER;

            return type::MOTOR_TYPE_UNKNOWN;
        }

        static std::string toString(type t)
        {
            switch(t) {
                case type::MOTOR_TYPE_STEPPER:
                    return "stepper";
                default:
                    return "unknown type (" + std::to_string((int)t) + ")";
                break;
            }
            return "";
        }
    };

    using StepperMotorType_t = StepperMotorType::type;

}

#endif
