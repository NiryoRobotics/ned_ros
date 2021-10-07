/*
fake_ttl_data.hpp
Copyright (C) 2020 Niryo
All rights reserved.
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http:// www.gnu.org/licenses/>.
*/

#ifndef FAKE_TTL_DATA_HPP
#define FAKE_TTL_DATA_HPP

#include <cstdint>
#include <string>
#include <vector>
namespace ttl_driver
{

class FakeTtlData
{
    public:
        FakeTtlData() {}
        ~FakeTtlData() {}
    public:
        struct AbstractFakeRegister
        {
            uint8_t id{0};
            uint16_t model_number{0};

            uint32_t position{0};
            uint32_t min_position{0};
            uint32_t max_position{0};

            uint32_t temperature{0};
            uint32_t voltage{0};
            std::string  firmware{};
        };

        struct FakeStepperRegister : public AbstractFakeRegister
        {
            uint32_t       velocity{0};
        };

        struct FakeDxlRegister : public AbstractFakeRegister
        {
            uint32_t       position_p_gain{0};
            uint32_t       position_i_gain{0};
            uint32_t       position_d_gain{0};

            uint32_t       velocity_p_gain{0};
            uint32_t       velocity_i_gain{0};

            uint32_t       ff1_gain{0};
            uint32_t       ff2_gain{0};
        };
        
        struct FakeEndEffector : public AbstractFakeRegister
        {
            uint32_t button1_action{0};
            uint32_t button2_action{0};
            uint32_t button3_action{0};
            
            uint32_t x_value{1};
            uint32_t y_value{1};
            uint32_t z_value{1};
            
            bool digitalInput = true;
            bool DigitalOutput = true;
        };

        // dxl
        std::vector<FakeDxlRegister> dxl_registers;

        // stepper
        std::vector<FakeStepperRegister> stepper_registers;

        // common
        std::vector<uint8_t> full_id_list;

        // end_effector
        FakeEndEffector end_effector;
};
}
#endif //FAKE_TTL_DATA_HPP
