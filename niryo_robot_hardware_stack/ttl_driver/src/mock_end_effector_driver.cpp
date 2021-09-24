#include "ttl_driver/mock_end_effector_driver.hpp"
#include "ttl_driver/end_effector_reg.hpp"
#include <cstddef>

namespace ttl_driver
{
// definition of methods

/**
 * @brief MockEndEffectorDriver::EndEffectorDriver
 */
MockEndEffectorDriver::MockEndEffectorDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                               std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractTtlDriver(portHandler, packetHandler)
{
}

/**
 * @brief MockEndEffectorDriver::~EndEffectorDriver
 */
MockEndEffectorDriver::~MockEndEffectorDriver()
{
}


//*****************************
// AbstractTtlDriver interface
//*****************************

/**
 * @brief MockEndEffectorDriver::str
 * @return
 */
std::string MockEndEffectorDriver::str() const
{
    return common::model::HardwareTypeEnum(EndEffectorReg::motor_type).toString() + " : " + AbstractTtlDriver::str();
}

/**
 * @brief MockEndEffectorDriver::interpreteErrorState
 * @return
 * TODO(CC) to be implemented
 */
std::string MockEndEffectorDriver::interpreteErrorState(uint32_t /*hw_state*/) const
{
    return "";
}

/**
 * @brief MockEndEffectorDriver::interpreteFirwmareVersion
 * @return
 */
std::string MockEndEffectorDriver::interpreteFirmwareVersion(uint32_t fw_version) const
{
    uint8_t v_major = static_cast<uint8_t>(fw_version >> 24);
    uint16_t v_minor = static_cast<uint16_t>(fw_version >> 8);
    uint8_t v_patch = static_cast<uint8_t>(fw_version >> 0);

    std::ostringstream ss;
    ss << std::to_string(v_major) << "."
       << std::to_string(v_minor) << "."
       << std::to_string(v_patch);
    std::string version = ss.str();

    return version;
}

/**
 * @brief MockEndEffectorDriver::ping
 * @return
 */
int MockEndEffectorDriver::ping(uint8_t id)
{
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::checkModelNumber
 * @param id
 * @return
 */
int MockEndEffectorDriver::checkModelNumber(uint8_t id)
{
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readFirmwareVersion
 * @param id
 * @param version
 * @return
 */
int MockEndEffectorDriver::readFirmwareVersion(uint8_t id, std::string &version)
{
    version = firmware_version;
    return COMM_SUCCESS;
}

// ram read

/**
 * @brief MockEndEffectorDriver::readTemperature
 * @param id
 * @param temperature
 * @return
 */
int MockEndEffectorDriver::readTemperature(uint8_t id, uint32_t& temperature)
{
    temperature = this->temperature;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readVoltage
 * @param id
 * @param voltage
 * @return
 */
int MockEndEffectorDriver::readVoltage(uint8_t id, double& voltage)
{
    voltage = static_cast<double>(voltage) / EndEffectorReg::VOLTAGE_CONVERSION;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readHwErrorStatus
 * @param id
 * @param hardware_status
 * @return
 */
int MockEndEffectorDriver::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    hardware_status = 0;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::syncReadFirmwareVersion
 * @param id_list
 * @param firmware_list
 * @return
 */
int MockEndEffectorDriver::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
{
    int res = 0;
    firmware_list.clear();
    for(size_t i = 0; i < id_list.size(); i++)
        firmware_list.emplace_back(firmware_version);
    return res;
}

/**
 * @brief MockEndEffectorDriver::syncReadTemperature
 * @param id_list
 * @param temperature_list
 * @return
 */
int MockEndEffectorDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    temperature_list.clear();
    for (size_t i = 0; i < id_list.size(); i++)
        temperature_list.emplace_back(temperature);
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::syncReadVoltage
 * @param id_list
 * @param voltage_list
 * @return
 */
int MockEndEffectorDriver::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
{
    voltage_list.clear();
    for(size_t i = 0; i < id_list.size(); i++)
        voltage_list.emplace_back(static_cast<double>(voltage) / EndEffectorReg::VOLTAGE_CONVERSION);
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::syncReadHwErrorStatus
 * @param id_list
 * @param hw_error_list
 * @return
 */
int MockEndEffectorDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
   // return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
    hw_error_list.clear();
    hw_error_list.emplace_back(0);
    return COMM_SUCCESS;
}

// buttons status

/**
 * @brief MockEndEffectorDriver::readButton1Status
 * @param id
 * @param action
 * @return
 */
int MockEndEffectorDriver::readButton1Status(uint8_t id,
                                                   common::model::EActionType& action)
{
    action = interpreteActionValue(_ee_info.button1_action);
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readButton2Status
 * @param id
 * @param action
 * @return
 */
int MockEndEffectorDriver::readButton2Status(uint8_t id, common::model::EActionType& action)
{
    action = interpreteActionValue(_ee_info.button2_action);
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readButton3Status
 * @param id
 * @param action
 * @return
 */
int MockEndEffectorDriver::readButton3Status(uint8_t id, common::model::EActionType& action)
{
    action = interpreteActionValue(_ee_info.button3_action);
    return COMM_SUCCESS;
}

// accelerometers and collision

/**
 * @brief MockEndEffectorDriver::readAccelerometerXValue
 * @param id
 * @param x_value
 * @return
 */
int MockEndEffectorDriver::readAccelerometerXValue(uint8_t id, uint32_t& x_value)
{
    x_value = _ee_info.x_value;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readAccelerometerYValue
 * @param id
 * @param y_value
 * @return
 */
int MockEndEffectorDriver::readAccelerometerYValue(uint8_t id, uint32_t& y_value)
{
    y_value = _ee_info.y_value;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readAccelerometerZValue
 * @param id
 * @param z_value
 * @return
 */
int MockEndEffectorDriver::readAccelerometerZValue(uint8_t id, uint32_t& z_value)
{
    z_value = _ee_info.z_value;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::readCollisionStatus
 * @param id
 * @param status
 * @return
 */
int MockEndEffectorDriver::readCollisionStatus(uint8_t id, bool& status)
{
    status = false;
    std::cout << "MockEndEffectorDriver::readCollisionStatus: need to be implemented!" << std::endl;
    return 0;
}

/**
 * @brief MockEndEffectorDriver::readDigitalInput
 * @param id
 * @param in
 * @return
 */
int MockEndEffectorDriver::readDigitalInput(uint8_t id, bool& in)
{
    in = _ee_info.digitalInput;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::setDigitalOutput
 * @param id
 * @param out
 * @return
 */
int MockEndEffectorDriver::writeDigitalOutput(uint8_t id, bool out)
{
    _ee_info.DigitalOutput = out;
    return COMM_SUCCESS;
}

/**
 * @brief MockEndEffectorDriver::interpreteActionValue
 * @param value
 * @return
 */
common::model::EActionType
MockEndEffectorDriver::interpreteActionValue(uint32_t value)
{
  common::model::EActionType action = common::model::EActionType::NO_ACTION;

  // HANDLE HELD en premier car c'est le seul cas ou il peut etre actif en meme temps qu'une autre action (long push)
  
  if (value & 1<<0)    // 0b00000001
  {
    action = common::model::EActionType::SINGLE_PUSH_ACTION;
  }
  else if (value & 1<<1)    // 0b00000010
  {
    action = common::model::EActionType::DOUBLE_PUSH_ACTION;
  }
  else if (value & 1<<2)    // 0b0000100
  {
    action = common::model::EActionType::LONG_PUSH_ACTION;
  }
  else if (value & 1<<3)    // 0b00001000
  {
    action = common::model::EActionType::HANDLE_HELD_ACTION;
  }
  return action;
}

/**
 * @brief MockEndEffectorDriver::writeSingleCmd
 * @param cmd
 * @return
 */
int MockEndEffectorDriver::writeSingleCmd(const std::shared_ptr<common::model::AbstractTtlSingleMotorCmd> &cmd)
{
  if (cmd && cmd->isValid())
  {
    switch (common::model::EEndEffectorCommandType(cmd->getCmdType()))
    {
        case common::model::EEndEffectorCommandType::CMD_TYPE_DIGITAL_OUTPUT:
            writeDigitalOutput(cmd->getId(), cmd->getParam());
            break;
        case common::model::EEndEffectorCommandType::CMD_TYPE_PING:
            ping(cmd->getId());
            break;
        default:
            std::cout << "Command not implemented" << std::endl;
    }
  }

  return 0;
}

/**
 * @brief MockEndEffectorDriver::writeSyncCmd
 * @return
 */
int MockEndEffectorDriver::writeSyncCmd(int /*type*/, const std::vector<uint8_t>& /*ids*/, const std::vector<uint32_t>& /*params*/)
{
  std::cout << "Synchronized cmd not implemented for end effector" << std::endl;

  return 0;
}
}
