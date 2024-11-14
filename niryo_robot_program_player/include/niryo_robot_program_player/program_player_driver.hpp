/*
program_player_driver.hpp
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

#ifndef NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_DRIVER_HPP
#define NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "niryo_robot_program_player/program_player_reg.hpp"
#include "common/model/action_type_enum.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace niryo_robot_program_player
{

constexpr int PROTOCOL_VERSION = 2;

template <typename reg_type = niryo_robot_program_player::ProgramPlayerReg>
class ProgramPlayerDriver
{
public:
  void init(const int& baudrate, const std::string& port);
  void closePort();

  std::string str() const;

  int readFirmwareVersion(uint8_t id, std::string& version);
  int ping(uint8_t id);

  int readStateButtonPlay(uint8_t id, common::model::EActionType& action);
  int readStateButtonStop(uint8_t id, common::model::EActionType& action);
  int readStateButtonCustom(uint8_t id, common::model::EActionType& action);
  int readStateButtonUp(uint8_t id, common::model::EActionType& action);
  int readStateButtonDown(uint8_t id, common::model::EActionType& action);

  int writeLCDLine1(uint8_t id, std::string& display_text);
  int writeLCDLine2(uint8_t id, std::string& display_text);

private:
  common::model::EActionType interpretActionValue(uint32_t value) const;
  std::string interpretFirmwareVersion(const std::array<uint8_t, 8>& data);

  template <typename T>
  int read(uint16_t address, uint8_t id, T& data);

  template <typename T>
  int write(uint16_t address, uint8_t id, T data);

private:
  dynamixel::PortHandler* _dxlPortHandler;
  dynamixel::PacketHandler* _dxlPacketHandler;

  bool _is_port_open = false;
};

// definition of methods

template <typename reg_type>
void ProgramPlayerDriver<reg_type>::init(const int& baudrate, const std::string& port)
{
  if (_is_port_open)
  {
    closePort();
  }

  try
  {
    _dxlPortHandler = dynamixel::PortHandler::getPortHandler(port.c_str());
    _dxlPacketHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (_dxlPortHandler && _dxlPortHandler->openPort())
    {
      if (_dxlPortHandler->setBaudRate(baudrate))
      {
        _dxlPortHandler->clearPort();
        _is_port_open = true;
      }
    }
  }
  catch (const std::exception& e)
  {
    // std::cerr << e.what() << '\n';
    delete _dxlPortHandler;
  }
}

template <typename reg_type>
void ProgramPlayerDriver<reg_type>::closePort()
{
  if (_is_port_open)
  {
    _dxlPortHandler->closePort();
    _is_port_open = false;
  }
}

template <typename reg_type>
std::string ProgramPlayerDriver<reg_type>::str() const
{
  return common::model::HardwareTypeEnum(reg_type::hardware_type).toString();
}

template <typename reg_type>
int ProgramPlayerDriver<reg_type>::readFirmwareVersion(uint8_t id, std::string& version)
{
  int res = COMM_RX_FAIL;
  uint32_t data{};
  res = read<typename reg_type::TYPE_FIRMWARE_VERSION>(reg_type::ADDR_FIRMWARE_VERSION, id, data);
  version = interpretFirmwareVersion(data);
  return res;
}

template <typename reg_type>
int ProgramPlayerDriver<reg_type>::ping(uint8_t id)
{
  if (!_is_port_open)
    return COMM_NOT_AVAILABLE;

  try
  {
    uint8_t dxl_error = 0;
    return _dxlPacketHandler->ping(_dxlPortHandler, id, &dxl_error);
  }
  catch (const std::exception& e)
  {
    // std::cerr << e.what() << '\n';
  }

  return COMM_NOT_AVAILABLE;
}

// buttons status

template <typename reg_type>
int ProgramPlayerDriver<reg_type>::readStateButtonPlay(uint8_t id, common::model::EActionType& action)
{
  uint8_t status;
  int res = read<typename reg_type::TYPE_BUTTON_STATUS>(reg_type::ADDR_STATE_BUTTON_PLAY, id, status);
  action = interpretActionValue(status);
  return res;
}

template <typename reg_type>
int ProgramPlayerDriver<reg_type>::readStateButtonStop(uint8_t id, common::model::EActionType& action)
{
  uint8_t status;
  int res = read<typename reg_type::TYPE_BUTTON_STATUS>(reg_type::ADDR_STATE_BUTTON_STOP, id, status);
  action = interpretActionValue(status);
  return res;
}

template <typename reg_type>
int ProgramPlayerDriver<reg_type>::readStateButtonCustom(uint8_t id, common::model::EActionType& action)
{
  uint8_t status;
  int res = read<typename reg_type::TYPE_BUTTON_STATUS>(reg_type::ADDR_STATE_BUTTON_CUSTOM, id, status);
  action = interpretActionValue(status);
  return res;
}

template <typename reg_type>
int ProgramPlayerDriver<reg_type>::readStateButtonUp(uint8_t id, common::model::EActionType& action)
{
  uint8_t status;
  int res = read<typename reg_type::TYPE_BUTTON_STATUS>(reg_type::ADDR_STATE_BUTTON_UP, id, status);
  action = interpretActionValue(status);
  return res;
}

template <typename reg_type>
int ProgramPlayerDriver<reg_type>::readStateButtonDown(uint8_t id, common::model::EActionType& action)
{
  uint8_t status;
  int res = read<typename reg_type::TYPE_BUTTON_STATUS>(reg_type::ADDR_STATE_BUTTON_DOWN, id, status);
  action = interpretActionValue(status);
  return res;
}

template <typename reg_type>
int ProgramPlayerDriver<reg_type>::writeLCDLine1(uint8_t id, std::string& display_text)
{
  if (!_is_port_open)
    return COMM_NOT_AVAILABLE;

  try
  {
    return _dxlPacketHandler->writeTxRx(_dxlPortHandler, id, reg_type::ADDR_LCD_LINE_1, 16,
                                        reinterpret_cast<uint8_t*>(const_cast<char*>(display_text.c_str())));
  }
  catch (const std::exception& e)
  {
    // std::cerr << e.what() << '\n';
  }

  return COMM_NOT_AVAILABLE;
}

template <typename reg_type>
int ProgramPlayerDriver<reg_type>::writeLCDLine2(uint8_t id, std::string& display_text)
{
  if (!_is_port_open)
    return COMM_NOT_AVAILABLE;

  try
  {
    return _dxlPacketHandler->writeTxRx(_dxlPortHandler, id, reg_type::ADDR_LCD_LINE_2, 16,
                                        reinterpret_cast<uint8_t*>(const_cast<char*>(display_text.c_str())));
  }
  catch (const std::exception& e)
  {
    // std::cerr << e.what() << '\n';
  }

  return COMM_NOT_AVAILABLE;
}

// READ/WRITE

// Helper functions to read/write data from/to the device using the DXL SDK
template <typename T>
int dxlRead(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler, uint8_t id,
            uint16_t address, T* data, uint8_t* error);

template <typename T>
int dxlWrite(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler, uint8_t id,
             uint16_t address, T data, uint8_t* error);

int dxlRead(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler, uint8_t id,
            uint16_t address, uint8_t* data, uint8_t* error)
{
  return packet_handler->read1ByteTxRx(port_handler, id, address, data, error);
}

int dxlRead(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler, uint8_t id,
            uint16_t address, uint16_t* data, uint8_t* error)
{
  return packet_handler->read2ByteTxRx(port_handler, id, address, data, error);
}

int dxlRead(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler, uint8_t id,
            uint16_t address, uint32_t* data, uint8_t* error)
{
  return packet_handler->read4ByteTxRx(port_handler, id, address, data, error);
}

int dxlWrite(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler, uint8_t id,
             uint16_t address, uint8_t data, uint8_t* error)
{
  return packet_handler->write1ByteTxRx(port_handler, id, address, data, error);
}

int dxlWrite(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler, uint8_t id,
             uint16_t address, uint16_t data, uint8_t* error)
{
  return packet_handler->write2ByteTxRx(port_handler, id, address, data, error);
}

int dxlWrite(dynamixel::PacketHandler* packet_handler, dynamixel::PortHandler* port_handler, uint8_t id,
             uint16_t address, uint32_t data, uint8_t* error)
{
  return packet_handler->write4ByteTxRx(port_handler, id, address, data, error);
}

template <typename reg_type>
template <typename T>
int ProgramPlayerDriver<reg_type>::read(uint16_t address, uint8_t id, T& data)
{
  int result{ COMM_NOT_AVAILABLE };
  uint8_t error = 0;

  if (!_is_port_open)
    return COMM_NOT_AVAILABLE;

  try
  {
    result = dxlRead(_dxlPacketHandler, _dxlPortHandler, id, address, &data, &error);
  }
  catch (const std::exception& e)
  {
    // std::cerr << e.what() << '\n';
  }

  if (error)
  {
    std::cerr << "read ERROR: device return error: id=" << id << " addr=" << address
              << " len=" << static_cast<int>(sizeof(T)) << " err=" << error << '\n';
  }

  return result;
}

template <typename reg_type>
template <typename T>
int ProgramPlayerDriver<reg_type>::write(uint16_t address, uint8_t id, T data)
{
  int result{ COMM_NOT_AVAILABLE };
  uint8_t error = 0;

  if (!_is_port_open)
    return COMM_NOT_AVAILABLE;

  try
  {
    result = dxlWrite(_dxlPacketHandler, _dxlPortHandler, id, address, data, &error);
  }
  catch (const std::exception& e)
  {
    // std::cerr << e.what() << '\n';
  }

  if (error)
  {
    std::cerr << "write ERROR: device return error: id=" << id << " addr=" << address
              << " len=" << static_cast<int>(sizeof(T)) << " err=" << error << '\n';
  }

  return result;
}

// READ/WRITE

template <typename reg_type>
common::model::EActionType ProgramPlayerDriver<reg_type>::interpretActionValue(uint32_t value) const
{
  common::model::EActionType action = common::model::EActionType::NO_ACTION;

  // HANDLE HELD en premier car c'est le seul cas ou il peut etre actif en meme temps qu'une autre action (long push)

  if (value & 1 << 0)  // 0b00000001
  {
    action = common::model::EActionType::SINGLE_PUSH_ACTION;
  }
  else if (value & 1 << 1)  // 0b00000010
  {
    action = common::model::EActionType::DOUBLE_PUSH_ACTION;
  }
  else if (value & 1 << 2)  // 0b0000100
  {
    action = common::model::EActionType::LONG_PUSH_ACTION;
  }
  else if (value & 1 << 3)  // 0b00001000
  {
    action = common::model::EActionType::HANDLE_HELD_ACTION;
  }
  return action;
}

template <typename reg_type>
std::string ProgramPlayerDriver<reg_type>::interpretFirmwareVersion(const std::array<uint8_t, 8>& data)
{
  int v_major = data[1];
  int v_minor = data[2];
  int v_patch = data[3];
  std::ostringstream ss;
  ss << v_major << "." << v_minor << "." << v_patch;
  std::string version = ss.str();

  return version;
}

}  // namespace niryo_robot_program_player

#endif  // NIRYO_ROBOT_PROGRAM_PLAYER_PROGRAM_PLAYER_DRIVER_HPP
