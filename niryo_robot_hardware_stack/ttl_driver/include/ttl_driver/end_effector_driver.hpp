/*
end_effector_driver.hpp
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

#ifndef END_EFFECTOR_DRIVER_HPP
#define END_EFFECTOR_DRIVER_HPP

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <cassert>

#include "abstract_motor_driver.hpp"

#include "end_effector_reg.hpp"
#include "common/model/end_effector_command_type_enum.hpp"
#include "common/model/end_effector_state.hpp"

using ::common::model::EEndEffectorCommandType;

namespace ttl_driver
{

/**
 * @brief The EndEffectorDriver class
 */
template<typename reg_type = EndEffectorReg>
class EndEffectorDriver : public AbstractTtlDriver
{
    public:
        EndEffectorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                          std::shared_ptr<dynamixel::PacketHandler> packetHandler);
        virtual ~EndEffectorDriver() override;

    public:
        // AbstractTtlDriver interface : we cannot define them globally in AbstractTtlDriver
        // as it is needed here for polymorphism (AbstractTtlDriver cannot be a template class and does not
        // have access to reg_type). So it seems like a duplicate of StepperDriver
        virtual std::string str() const override;

        virtual std::string interpreteErrorState(uint32_t hw_state) override;
        virtual int checkModelNumber(uint8_t id) override;
        virtual int readFirmwareVersion(uint8_t id, uint32_t &version) override;
        
        virtual int readTemperature(uint8_t id, uint32_t &temperature) override;
        virtual int readVoltage(uint8_t id, uint32_t &voltage) override;
        virtual int readHwErrorStatus(uint8_t id, uint32_t &hardware_status) override;
        
        virtual int syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) override;
        virtual int syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) override;
        virtual int syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) override;

        virtual int writeSingleCmd(std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> &cmd) override;
        virtual int writeSyncCmd(int type, const std::vector<uint8_t>& ids, const std::vector<uint32_t>& params) override;

    public:
        int setButton1Configuration(uint8_t id, uint32_t configuration);
        int setButton2Configuration(uint8_t id, uint32_t configuration);
        int setButton3Configuration(uint8_t id, uint32_t configuration);

        int readButton1Configuration(uint8_t id, uint32_t& configuration);
        int readButton2Configuration(uint8_t id, uint32_t& configuration);
        int readButton3Configuration(uint8_t id, uint32_t& configuration);

        int readButton1Status(uint8_t id, uint32_t& status);
        int readButton2Status(uint8_t id, uint32_t& status);
        int readButton3Status(uint8_t id, uint32_t& status);

        int readAccelerometerXValue(uint8_t id, uint32_t& x_value);
        int readAccelerometerYValue(uint8_t id, uint32_t& y_value);
        int readAccelerometerZValue(uint8_t id, uint32_t& z_value);

        int readCollisionStatus(uint8_t id, bool&status);

        int readDigitalInput(uint8_t id, bool& in);
        int setDigitalOutput(uint8_t id, bool out);

        common::model::EndEffectorState::EActionType interpreteActionValue(uint32_t value);
};

// definition of methods

/**
 * @brief EndEffectorDriver<reg_type>::EndEffectorDriver
 */
template<typename reg_type>
EndEffectorDriver<reg_type>::EndEffectorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                               std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractTtlDriver(portHandler, packetHandler)
{
}

/**
 * @brief EndEffectorDriver<reg_type>::~EndEffectorDriver
 */
template<typename reg_type>
EndEffectorDriver<reg_type>::~EndEffectorDriver()
{
}


//*****************************
// AbstractTtlDriver interface
//*****************************

/**
 * @brief EndEffectorDriver<reg_type>::str
 * @return
 */
template<typename reg_type>
std::string EndEffectorDriver<reg_type>::str() const
{
    return common::model::HardwareTypeEnum(reg_type::motor_type).toString() + " : " + AbstractTtlDriver::str();
}

/**
 * @brief EndEffectorDriver<reg_type>::interpreteErrorState
 * @return
 */
template<typename reg_type>
std::string EndEffectorDriver<reg_type>::interpreteErrorState(uint32_t /*hw_state*/)
{
    return "no error table";
}

/**
 * @brief EndEffectorDriver<reg_type>::checkModelNumber
 * @param id
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::checkModelNumber(uint8_t id)
{
    uint16_t model_number = 0;
    int ping_result = getModelNumber(id, model_number);

    if (ping_result == COMM_SUCCESS)
    {
        if (model_number && model_number != reg_type::MODEL_NUMBER)
        {
            return PING_WRONG_MODEL_NUMBER;
        }
    }

    return ping_result;
}

/**
 * @brief EndEffectorDriver<reg_type>::readFirmwareVersion
 * @param id
 * @param version
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readFirmwareVersion(uint8_t id, uint32_t &version)
{
    return read(reg_type::ADDR_FIRMWARE_VERSION, reg_type::SIZE_FIRMWARE_VERSION, id, version);
}

// ram read

/**
 * @brief EndEffectorDriver<reg_type>::readTemperature
 * @param id
 * @param temperature
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readTemperature(uint8_t id, uint32_t& temperature)
{
    return read(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id, temperature);
}

/**
 * @brief EndEffectorDriver<reg_type>::readVoltage
 * @param id
 * @param voltage
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readVoltage(uint8_t id, uint32_t& voltage)
{
    return read(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id, voltage);
}

/**
 * @brief EndEffectorDriver<reg_type>::readHwErrorStatus
 * @param id
 * @param hardware_status
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    hardware_status = 0;
    std::cout << "readHwErrorStatus not yet implemented" << std::endl;
    //return read(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id, hardware_status);
    return COMM_RX_FAIL;
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadTemperature
 * @param id_list
 * @param temperature_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    return syncRead(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id_list, temperature_list);
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadVoltage
 * @param id_list
 * @param voltage_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)
{
    return syncRead(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id_list, voltage_list);
}

/**
 * @brief EndEffectorDriver<reg_type>::syncReadHwErrorStatus
 * @param id_list
 * @param hw_error_list
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    std::cout << "readHwErrorStatus not yet implemented" << std::endl;

   // return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
    return COMM_RX_FAIL;
}

// buttons configuration

/**
 * @brief EndEffectorDriver<reg_type>::setButton1Configuration
 * @param id
 * @param configuration
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::setButton1Configuration(uint8_t id, uint32_t configuration)
{
    return write(reg_type::ADDR_BUTTON_1_CONFIG, reg_type::SIZE_BUTTON_1_CONFIG, id, configuration);
}

/**
 * @brief EndEffectorDriver<reg_type>::setButton2Configuration
 * @param id
 * @param configuration
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::setButton2Configuration(uint8_t id, uint32_t configuration)
{
    return write(reg_type::ADDR_BUTTON_2_CONFIG, reg_type::SIZE_BUTTON_2_CONFIG, id, configuration);
}

/**
 * @brief EndEffectorDriver<reg_type>::setButton3Configuration
 * @param id
 * @param configuration
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::setButton3Configuration(uint8_t id, uint32_t configuration)
{
    return write(reg_type::ADDR_BUTTON_3_CONFIG, reg_type::SIZE_BUTTON_3_CONFIG, id, configuration);
}

/**
 * @brief EndEffectorDriver<reg_type>::readButton1Configuration
 * @param id
 * @param configuration
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readButton1Configuration(uint8_t id, uint32_t& configuration)
{
    return read(reg_type::ADDR_BUTTON_1_CONFIG, reg_type::SIZE_BUTTON_1_CONFIG, id, configuration);
}
    
/**
 * @brief EndEffectorDriver<reg_type>::readButton2Configuration
 * @param id
 * @param configuration
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readButton2Configuration(uint8_t id, uint32_t& configuration)
{
    return read(reg_type::ADDR_BUTTON_2_CONFIG, reg_type::SIZE_BUTTON_2_CONFIG, id, configuration);
}

/**
 * @brief EndEffectorDriver<reg_type>::readButton3Configuration
 * @param id
 * @param configuration
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readButton3Configuration(uint8_t id, uint32_t& configuration)
{
    return read(reg_type::ADDR_BUTTON_3_CONFIG, reg_type::SIZE_BUTTON_3_CONFIG, id, configuration);
}

// buttons status

/**
 * @brief EndEffectorDriver<reg_type>::readButton1Status
 * @param id
 * @param status
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readButton1Status(uint8_t id, uint32_t& status)
{
    return read(reg_type::ADDR_BUTTON_1_STATUS, reg_type::SIZE_BUTTON_1_STATUS, id, status);
}

/**
 * @brief EndEffectorDriver<reg_type>::readButton2Status
 * @param id
 * @param status
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readButton2Status(uint8_t id, uint32_t& status)
{
    return read(reg_type::ADDR_BUTTON_2_STATUS, reg_type::SIZE_BUTTON_2_STATUS, id, status);
}

/**
 * @brief EndEffectorDriver<reg_type>::readButton3Status
 * @param id
 * @param status
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readButton3Status(uint8_t id, uint32_t& status)
{
    return read(reg_type::ADDR_BUTTON_3_STATUS, reg_type::SIZE_BUTTON_3_STATUS, id, status);
}

// accelerometers and collision

/**
 * @brief EndEffectorDriver<reg_type>::readAccelerometerXValue
 * @param id
 * @param x_value
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readAccelerometerXValue(uint8_t id, uint32_t& x_value)
{
    return read(reg_type::ADDR_ACCELERO_VALUE_X, reg_type::SIZE_ACCELERO_VALUE_X, id, x_value);
}

/**
 * @brief EndEffectorDriver<reg_type>::readAccelerometerYValue
 * @param id
 * @param y_value
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readAccelerometerYValue(uint8_t id, uint32_t& y_value)
{
    return read(reg_type::ADDR_ACCELERO_VALUE_Y, reg_type::SIZE_ACCELERO_VALUE_Y, id, y_value);
}

/**
 * @brief EndEffectorDriver<reg_type>::readAccelerometerZValue
 * @param id
 * @param z_value
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readAccelerometerZValue(uint8_t id, uint32_t& z_value)
{
    return read(reg_type::ADDR_ACCELERO_VALUE_Z, reg_type::SIZE_ACCELERO_VALUE_Z, id, z_value);
}

/**
 * @brief EndEffectorDriver<reg_type>::readCollisionStatus
 * @param id
 * @param status
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readCollisionStatus(uint8_t id, bool& status)
{
    status = false;
    std::cout << "EndEffectorDriver<reg_type>::readCollisionStatus: need to be implemented!" << std::endl;
    return 0;
}

/**
 * @brief EndEffectorDriver<reg_type>::readDigitalInput
 * @param id
 * @param in
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::readDigitalInput(uint8_t id, bool& in)
{
    uint32_t value;
    int res = read(reg_type::ADDR_DIGITAL_IN, reg_type::SIZE_DIGITAL_IN, id, value);
    in = (value > 0) ? true : false;
    return res;
}

/**
 * @brief EndEffectorDriver<reg_type>::setDigitalOutput
 * @param id
 * @param out
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::setDigitalOutput(uint8_t id, bool out)
{
    return read(reg_type::ADDR_DIGITAL_OUT, reg_type::SIZE_DIGITAL_OUT, id, (out > 0) ? 1 : 0);
}

/**
 * @brief EndEffectorDriver<reg_type>::interpreteActionValue
 * @param value
 * @return
 */
template<typename reg_type>
common::model::EndEffectorState::EActionType
EndEffectorDriver<reg_type>::interpreteActionValue(uint32_t value)
{
  common::model::EndEffectorState::EActionType action = common::model::EndEffectorState::EActionType::NO_ACTION;

  if (value & 1<<0)    // 0b00000001
  {
      action = common::model::EndEffectorState::EActionType::HANDLE_HELD_ACTION;
  }
  else if (value & 1<<2)    // 0b00000100
  {
    action = common::model::EndEffectorState::EActionType::SINGLE_PUSH_ACTION;
  }
  else if (value & 1<<3)    // 0b00001000
  {
    action = common::model::EndEffectorState::EActionType::DOUBLE_PUSH_ACTION;
  }
  else if (value & 1<<4)    // 0b00010000
  {
    action = common::model::EndEffectorState::EActionType::LONG_PUSH_ACTION;
  }

  return action;
}

/**
 * @brief EndEffectorDriver<reg_type>::writeSingleCmd
 * @param cmd
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::writeSingleCmd(std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> &cmd)
{
  if (cmd && cmd->isValid())
  {
      switch (EEndEffectorCommandType(cmd->getCmdType()))
      {
      case EEndEffectorCommandType::CMD_TYPE_BUTTON_1_CONFIG:
          return setButton1Configuration(cmd->getId(), cmd->getParam());
      case EEndEffectorCommandType::CMD_TYPE_BUTTON_2_CONFIG:
          return setButton2Configuration(cmd->getId(), cmd->getParam());
      case EEndEffectorCommandType::CMD_TYPE_BUTTON_3_CONFIG:
          return setButton3Configuration(cmd->getId(), cmd->getParam());
      default:
          std::cout << "Command not implemented" << std::endl;
      }
  }

  return 0;
}

/**
 * @brief EndEffectorDriver<reg_type>::writeSyncCmd
 * @return
 */
template<typename reg_type>
int EndEffectorDriver<reg_type>::writeSyncCmd(int /*type*/, const std::vector<uint8_t>& /*ids*/, const std::vector<uint32_t>& /*params*/)
{
  std::cout << "Synchronized cmd not implemented for end effector" << std::endl;

  return 0;
}


} // ttl_driver

#endif // EndEffectorDriver
