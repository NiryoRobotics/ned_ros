#include "xl430_reg.hpp"

namespace ttl_driver
{
    struct XH430Reg : XL430Reg
    {
        static constexpr int MODEL_NUMBER = 1010;

        static constexpr uint16_t ADDR_CURRENT_LIMIT = 38;
        using TYPE_CURRENT_LIMIT = uint16_t;

        static constexpr uint16_t ADDR_GOAL_CURRENT = 102;
        using TYPE_GOAL_CURRENT = uint16_t;

        static constexpr uint16_t ADDR_PRESENT_CURRENT = 126;
        using TYPE_PRESENT_CURRENT = uint16_t;
    };
}