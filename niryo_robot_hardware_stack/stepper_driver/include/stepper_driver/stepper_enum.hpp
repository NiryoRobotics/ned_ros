#ifndef STEPPER_ENUM_HPP
#define STEPPER_ENUM_HPP

#include <string>

namespace StepperDriver
{
    struct StepperCommandType {
        enum class type
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
            CMD_TYPE_UNKNOWN=100
        };

        static std::string toString(type t)
        {
            switch(t) {
                case type::CMD_TYPE_NONE:
                    return "none";
                case type::CMD_TYPE_POSITION:
                    return "position";
                case type::CMD_TYPE_VELOCITY:
                    return "velocity";
                case type::CMD_TYPE_EFFORT:
                    return "effort";
                case type::CMD_TYPE_TORQUE:
                    return "torque";
                case type::CMD_TYPE_SYNCHRONIZE:
                    return "nonsynchronize";
                case type::CMD_TYPE_RELATIVE_MOVE:
                    return "relative move";
                case type::CMD_TYPE_MAX_EFFORT:
                    return "max effort";
                case type::CMD_TYPE_MICRO_STEPS:
                    return "micro steps";
                case type::CMD_TYPE_POSITION_OFFSET:
                    return "position offset";
                case type::CMD_TYPE_CALIBRATION:
                    return "calibration";
                case type::CMD_TYPE_CONVEYOR:
                    return "conveyor";
                case type::CMD_TYPE_UPDATE_CONVEYOR:
                    return "update conveyor";
                case type::CMD_TYPE_LEARNING_MODE:
                    return "learning mode";
                case type::CMD_TYPE_UNKNOWN:
                default:
                    return "unknown type (" + std::to_string(static_cast<int>(t)) + ")";
            }
        }
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
                    return "unknown type (" + std::to_string(static_cast<int>(t)) + ")";
            }
        }
    };

    using StepperCommandType_t = StepperCommandType::type;
    using StepperMotorType_t = StepperMotorType::type;

}

#endif
