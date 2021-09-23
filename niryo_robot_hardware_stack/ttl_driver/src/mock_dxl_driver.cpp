#include <ttl_driver/mock_dxl_driver.hpp>

namespace ttl_driver
{
/**
 * @brief MockDxlDriver::MockDxlDriver
 */
MockDxlDriver::MockDxlDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                               std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
    AbstractDxlDriver(portHandler, packetHandler)
{
    // retrieve list of ids
    for(auto const& imap: _map_fake_registers)
        _id_list.emplace_back(imap.first);
}

MockDxlDriver::~MockDxlDriver()
{
}

std::string MockDxlDriver::str() const
{
  return "Mock Dynamixel Driver (OK)";
}

//*****************************
// AbstractTtlDriver interface
//*****************************

int MockDxlDriver::ping(uint8_t id)
{
    return _map_fake_registers.count(id) ? COMM_SUCCESS : COMM_TX_FAIL;
}

int MockDxlDriver::getModelNumber(uint8_t id, uint16_t& model_number)
{
    model_number = _map_fake_registers.at(id).model_number;
    return COMM_SUCCESS;
}

int MockDxlDriver::scan(std::vector<uint8_t>& id_list)
{
    id_list = _full_id_list;
    return COMM_SUCCESS;
}

int MockDxlDriver::reboot(uint8_t id)
{
    if (std::find(_id_list.begin(), _id_list.end(), id) == _id_list.end())
        return COMM_TX_FAIL;
    return COMM_SUCCESS;
}

std::string MockDxlDriver::interpreteErrorState(uint32_t /*hw_state*/) const
{
    return "";
}

int MockDxlDriver::changeId(uint8_t id, uint8_t new_id)
{
    return COMM_TX_FAIL;
}

int MockDxlDriver::checkModelNumber(uint8_t id)
{
    uint16_t model_number = 0;
    int ping_result = getModelNumber(id, model_number);

    if (ping_result == COMM_SUCCESS)
    {
        if (model_number && model_number != 0)
        {
            return PING_WRONG_MODEL_NUMBER;
        }
    }

    return ping_result;
}

int MockDxlDriver::readFirmwareVersion(uint8_t id, std::string &version)
{
    version = _map_fake_registers.at(id).firmware;
    return COMM_SUCCESS;
}

int MockDxlDriver::readMinPosition(uint8_t id, uint32_t &pos)
{
    pos = _map_fake_registers.at(id).min_position;
    return COMM_SUCCESS;
}

int MockDxlDriver::readMaxPosition(uint8_t id, uint32_t &pos)
{
    pos = _map_fake_registers.at(id).max_position;
    return COMM_SUCCESS;
}

// ram write

int MockDxlDriver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setGoalPosition(uint8_t id, uint32_t position)
{
    _map_fake_registers.at(id).position = position;
    return COMM_SUCCESS;
}

int MockDxlDriver::setGoalVelocity(uint8_t id, uint32_t velocity)
{
    // in mode control Position Control Mode, velocity profile in datasheet is used to set velocity (except xl320)
    return COMM_SUCCESS;
}

int MockDxlDriver::syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list)
{
    // Create a map to store the frequency of each element in vector
    std::map<uint8_t, uint8_t> countMap;
    // Iterate over the vector and store the frequency of each element in map
    for (auto & id : id_list)
    {
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list)
{
    if (id_list.size() != position_list.size())
        return LEN_ID_DATA_NOT_SAME;

    // Create a map to store the frequency of each element in vector
    std::map<uint8_t, uint8_t> countMap;    
    for (size_t i = 0; i < id_list.size(); i++)
    {
        _map_fake_registers.at(id_list.at(i)).position = position_list.at(i);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id_list[i], 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list)
{
    // Create a map to store the frequency of each element in vector
    std::map<uint8_t, uint8_t> countMap;
    // Iterate over the vector and store the frequency of each element in map
    for (auto & id : id_list)
    {
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

// ram read

int MockDxlDriver::readPosition(uint8_t id, uint32_t& present_position)
{
    present_position = _map_fake_registers.at(id).position;
    return COMM_SUCCESS;
}

int MockDxlDriver::readTemperature(uint8_t id, uint32_t& temperature)
{
    temperature = _map_fake_registers.at(id).temperature;
    return COMM_SUCCESS;
}

int MockDxlDriver::readVoltage(uint8_t id, double& voltage)
{
    voltage = _map_fake_registers.at(id).voltage;
    return COMM_SUCCESS;
}

int MockDxlDriver::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    hardware_status = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        position_list.emplace_back(_map_fake_registers.at(id).position);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadFirmwareVersion(const std::vector<uint8_t> &id_list, std::vector<std::string> &firmware_list)
{
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        firmware_list.emplace_back(_map_fake_registers.at(id).firmware);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        temperature_list.emplace_back(_map_fake_registers.at(id).temperature);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<double> &voltage_list)
{
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        voltage_list.emplace_back(_map_fake_registers.at(id).voltage);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    std::map<uint8_t, uint8_t> countMap;
    for (auto & id : id_list)
    {
        hw_error_list.push_back(0);
        auto result = countMap.insert(std::pair<uint8_t, uint8_t>(id, 1));
        if (result.second == false)
            return GROUP_SYNC_REDONDANT_ID;  // redondant id
    }
    return COMM_SUCCESS;
}

//*****************************
// AbstractDxlDriver interface
//*****************************
int MockDxlDriver::setLed(uint8_t id, uint32_t led_value)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setGoalTorque(uint8_t id, uint32_t torque)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setPositionPGain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setPositionIGain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setPositionDGain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setVelocityPGain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setVelocityIGain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setff1Gain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::setff2Gain(uint8_t id, uint32_t gain)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::readLoad(uint8_t id, uint32_t& present_load)
{
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
{
    load_list = {};
    for (size_t i = 0; i < id_list.size(); i++)
        load_list.push_back(0);
    return COMM_SUCCESS;
}

int MockDxlDriver::readVelocity(uint8_t id, uint32_t& present_velocity)
{
    present_velocity = 0;
    return COMM_SUCCESS;
}

int MockDxlDriver::syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
{
    velocity_list = {};
    for (size_t i = 0; i < id_list.size(); i++)
        velocity_list.push_back(0);
    return COMM_SUCCESS;
}

std::string MockDxlDriver::interpreteFirmwareVersion(uint32_t fw_version) const
{
    return "";
}

void MockDxlDriver::removeGripper()
{
    _full_id_list.erase(std::remove(_full_id_list.begin(), _full_id_list.end(), 11), _full_id_list.end());
    _map_fake_registers.erase(11);
}
}  // namespace ttl_driver
