/*
program_player_reg.hpp
Copyright (C) 2024 Niryo
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

#ifndef NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_REG_HPP
#define NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_REG_HPP

#include <memory>
#include "common/model/hardware_type_enum.hpp"

namespace niryo_robot_program_player
{
struct ProgramPlayerReg
{
  static constexpr common::model::EHardwareType hardware_type = common::model::EHardwareType::PROGRAM_PLAYER;

  static constexpr float PROTOCOL_VERSION = 2.0;
  static constexpr int MODEL_NUMBER = 2102;

  // EEPROM

  static constexpr uint16_t ADDR_MODEL_NUMBER = 0;
  using TYPE_MODEL_NUMBER = uint16_t;

  static constexpr uint16_t ADDR_ID = 7;
  using TYPE_ID = uint8_t;

  static constexpr uint16_t ADDR_BAUDRATE = 8;
  using TYPE_BAUDRATE = uint8_t;

  static constexpr uint16_t ADDR_FIRMWARE_VERSION = 59;
  using TYPE_FIRMWARE_VERSION = uint32_t;

  // RAM

  static constexpr uint16_t ADDR_SERIAL_NUMBER = 2560;
  using TYPE_SERIAL_NUMBER = uint32_t;

  using TYPE_BUTTON_STATUS = uint8_t;
  static constexpr uint16_t ADDR_STATE_BUTTON_PLAY = 2564;

  static constexpr uint16_t ADDR_STATE_BUTTON_STOP = 2565;

  static constexpr uint16_t ADDR_STATE_BUTTON_CUSTOM = 2566;

  static constexpr uint16_t ADDR_STATE_BUTTON_UP = 2567;

  static constexpr uint16_t ADDR_STATE_BUTTON_DOWN = 2568;

  using TYPE_LCD_LINE = uint16_t;  // [2];
  static constexpr uint16_t ADDR_LCD_LINE_1 = 2569;

  static constexpr uint16_t ADDR_LCD_LINE_2 = 2585;

  // Firmware status

  static constexpr uint16_t ADDR_FIRMWARE_RUNNING = 8192;
  using TYPE_FIRMWARE_RUNNING = uint8_t;

  static constexpr uint16_t ADDR_ENTER_BOOTLOADER = 8193;
  using TYPE_ENTER_BOOTLOADER = uint32_t;

  static constexpr uint16_t ADDR_OTA_BEGIN = 8197;
  using TYPE_OTA_BEGIN = uint8_t;

  static constexpr uint16_t ADDR_OTA_WRITE = 8198;
  using TYPE_OTA_WRITE = uint8_t[520];

  static constexpr uint16_t ADDR_OTA_END = 8718;
  using TYPE_OTA_END = uint32_t;

  static constexpr uint16_t ADDR_OTA_ERASE = 8722;
  using TYPE_OTA_ERASE = uint32_t;
};
}  // namespace niryo_robot_program_player

#endif  // NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_REG_HPP
