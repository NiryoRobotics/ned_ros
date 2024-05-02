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
#include <map>
#include <vector>

namespace ttl_driver
{

class FakeTtlData
{
    public:
        FakeTtlData() = default;

        void updateFullIdList();

    public:
        struct AbstractFakeRegister
        {
            uint8_t id{0};
            uint16_t model_number{0};

            uint32_t position{0};
            uint32_t velocity{0};

            uint32_t min_position{0};
            uint32_t max_position{0};

            uint8_t temperature{0};
            double voltage{0};
            std::string  firmware{};
        };

        struct FakeStepperRegister : public AbstractFakeRegister
        {
            uint8_t torque{0};
            uint32_t v_start{1};
            uint32_t a_1{0};
            uint32_t v_1{0};
            uint32_t a_max{6000};
            uint32_t v_max{6};
            uint32_t d_max{6000};
            uint32_t d_1{0};
            uint32_t v_stop{2};
            int32_t  homing_abs_position{0};
            uint8_t  operating_mode{0};;
        };

        struct FakeDxlRegister : public AbstractFakeRegister
        {
            uint8_t        torque{0};
            uint16_t       position_p_gain{0};
            uint16_t       position_i_gain{0};
            uint16_t       position_d_gain{0};

            uint16_t       velocity_p_gain{0};
            uint16_t       velocity_i_gain{0};

            uint16_t       ff1_gain{0};
            uint16_t       ff2_gain{0};
        };
        
        struct FakeEndEffector : public AbstractFakeRegister
        {
            uint32_t button0_action{1};
            uint32_t button1_action{2};
            uint32_t button2_action{8};
            
            uint32_t x_value{1};
            uint32_t y_value{1};
            uint32_t z_value{1};
            
            bool digitalInput = true;
            bool DigitalOutput = true;
        };

        // dxl by id
        std::map<uint8_t, FakeDxlRegister> dxl_registers;

        // stepper by id
        std::map<uint8_t, FakeStepperRegister> stepper_registers;

        // all ids
        std::vector<uint8_t> full_id_list;

        // end_effector
        FakeEndEffector end_effector;
};

inline
void FakeTtlData::updateFullIdList()
{
    full_id_list.clear();
    for (auto const& it : dxl_registers)
    {
        full_id_list.emplace_back(it.first);
    }

    for (auto const& it : stepper_registers)
    {
        full_id_list.emplace_back(it.first);
    }

    if (!end_effector.firmware.empty())
        full_id_list.emplace_back(end_effector.id);
}

}
#endif //FAKE_TTL_DATA_HPP
