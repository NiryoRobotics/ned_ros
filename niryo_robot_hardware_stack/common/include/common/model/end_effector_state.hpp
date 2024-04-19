/*
end_effector_state.hpp
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

#ifndef END_EFFECTOR_STATE_H
#define END_EFFECTOR_STATE_H

#include "abstract_hardware_state.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <cstdint>
#include <string>
#include <cassert>
#include <sstream>
#include <queue>

#include "common/model/dxl_command_type_enum.hpp"
#include "hardware_type_enum.hpp"
#include "button_type_enum.hpp"
#include "action_type_enum.hpp"
#include "ros/time.h"

namespace common
{
namespace model
{

/**
 * @brief The EndEffectorState class
 */
class EndEffectorState : public AbstractHardwareState
{
    public:
        /**
         * @brief The Button class
         */
        class Button : public IObject
        {
            public:
                Button();

                std::string str() const override;
                bool isValid() const override;
                void reset() override;

                // need a delay to avoid state hand hold called when other states come
                void setDelay();

                // check if hand hold state came is needed to skip
                bool needsToSkip();

            public:
                EButtonType type{EButtonType::UNKNOWN};
                std::queue<EActionType> actions;

            private:
                static constexpr double _time_avoid_duplicate_state = 0.5;
                double _time_last_read_state{};
                bool _need_delay{false};
        };

        struct Vector3D
        {
          uint32_t x{};
          uint32_t y{};
          uint32_t z{};

          std::string str() const
          {
            std::ostringstream ss;
            ss << "(" << x << "," << y << "," << z << ")";
            return ss.str();
          }
        };

    public:
        EndEffectorState();
        EndEffectorState(uint8_t id);
        EndEffectorState(uint8_t id, common::model::EHardwareType type);

        void configureButton(uint8_t button_id, EButtonType button_type);

        // AbstractHardwareState interface
        std::string str() const override;

        // IObject interface
    public:
        bool isValid() const override;

    public:
        void setButtonStatus(uint8_t id, EActionType action);

        std::array<std::shared_ptr<Button>, 3> getButtonsStatus() const;

        uint32_t getAccelerometerXValue() const;
        uint32_t getAccelerometerYValue() const;
        uint32_t getAccelerometerZValue() const;

        void setAccelerometerXValue(const uint32_t& xValue);
        void setAccelerometerYValue(const uint32_t& yValue);
        void setAccelerometerZValue(const uint32_t& zValue);

        bool getCollisionStatus() const;
        void setCollisionStatus(bool status);
        int getCollisionThresh() const;
        void setCollisionThresh(int thresh);
        int getCollisionThreshAlgo2() const;
        void setCollisionThreshAlgo2(int thresh);

        bool getDigitalIn() const;
        void setDigitalIn(bool digital_in);

        bool getDigitalOut() const;
        void setDigitalOut(bool digital_out);

    private:
        std::array<std::shared_ptr<Button>, 3> _buttons_list{};
        Vector3D _accelerometer_values{};

        bool _collision_status{false};
        int _collision_thresh{0};
        int _collision_thresh_algo_2{0};

        bool _digital_in{false};
        bool _digital_out{false};
};

/**
 * @brief EndEffectorState::getButtonsStatus
 * @return
 */
inline
std::array<std::shared_ptr<EndEffectorState::Button>, 3>
EndEffectorState::getButtonsStatus() const
{
  return _buttons_list;
}

/**
 *
 * @brief EndEffectorState::getAccelerometerXValue
 * @return
 */
inline
uint32_t EndEffectorState::getAccelerometerXValue() const
{
  return _accelerometer_values.x;
}

/**
 * @brief EndEffectorState::getAccelerometerYValue
 * @return
 */
inline
uint32_t EndEffectorState::getAccelerometerYValue() const
{
  return _accelerometer_values.y;
}

/**
 * @brief EndEffectorState::getAccelerometerZValue
 * @return
 */
inline
uint32_t EndEffectorState::getAccelerometerZValue() const
{
  return _accelerometer_values.z;
}

/**
 * @brief EndEffectorState::getCollisionStatus
 * @return
 */
inline
bool EndEffectorState::getCollisionStatus() const
{
  return _collision_status;
}

/**
 * @brief EndEffectorState::getCollisionThresh
 * @return
 */
inline
int EndEffectorState::getCollisionThresh() const
{
  return _collision_thresh;
}

/**
 * @brief EndEffectorState::getCollisionThreshAlgo2
 * @return
 */
inline
int EndEffectorState::getCollisionThreshAlgo2() const
{
  return _collision_thresh_algo_2;
}

/**
 * @brief EndEffectorState::getDigitalIn
 * @return
 */
inline
bool EndEffectorState::getDigitalIn() const
{
  return _digital_in;
}

/**
 * @brief EndEffectorState::getDigitalOut
 * @return
 */
inline
bool EndEffectorState::getDigitalOut() const
{
  return _digital_out;
}

} // namespace model
} // namespace common

#endif // END_EFFECTOR_STATE_H
