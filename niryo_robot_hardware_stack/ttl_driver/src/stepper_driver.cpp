/*
    can_driver.cpp
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
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ttl_driver/stepper_driver.hpp"
#include "ttl_driver/stepper_reg.hpp"

#include <sstream>

using namespace std;

namespace ttl_driver
{

    StepperDriver::StepperDriver(std::shared_ptr<dynamixel::PortHandler> portHandler,
                                 std::shared_ptr<dynamixel::PacketHandler> packetHandler) :
        AbstractMotorDriver(portHandler, packetHandler)
    {

    }

    StepperDriver::~StepperDriver()
    {

    }

    std::string StepperDriver::str() const {
        return "Driver - type StepperDriver (TTL) \n" + AbstractMotorDriver::str();
    }

    std::string StepperDriver::interpreteErrorState(uint32_t hw_state)  { std::cout << "not implemented" << std::endl; return ""; }

    int StepperDriver::checkModelNumber(uint8_t id) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::changeId(uint8_t id, uint8_t new_id) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::changeBaudRate(uint8_t id, uint32_t new_baudrate) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setReturnDelayTime(uint8_t id, uint32_t return_delay_time) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setLimitTemperature(uint8_t id, uint32_t temperature) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setMaxTorque(uint8_t id, uint32_t torque) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setReturnLevel(uint8_t id, uint32_t return_level) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readReturnDelayTime(uint8_t id, uint32_t *return_delay_time) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readLimitTemperature(uint8_t id, uint32_t *limit_temperature) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readMaxTorque(uint8_t id, uint32_t *max_torque) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readReturnLevel(uint8_t id, uint32_t *return_level) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setTorqueEnable(uint8_t id, uint32_t torque_enable) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setLed(uint8_t id, uint32_t led_value) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setGoalPosition(uint8_t id, uint32_t position) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setGoalVelocity(uint8_t id, uint32_t velocity) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setGoalTorque(uint8_t id, uint32_t torque) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setPGain(uint8_t id, uint32_t gain) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setIGain(uint8_t id, uint32_t gain) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setDGain(uint8_t id, uint32_t gain) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setff1Gain(uint8_t id, uint32_t gain) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::setff2Gain(uint8_t id, uint32_t gain) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncWriteLed(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &led_list) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncWriteTorqueEnable(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_enable_list) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncWritePositionGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &position_list) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncWriteVelocityGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &velocity_list) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncWriteTorqueGoal(const std::vector<uint8_t> &id_list, const std::vector<uint32_t> &torque_list) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readPosition(uint8_t id, uint32_t *present_position) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readVelocity(uint8_t id, uint32_t *present_velocity) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readLoad(uint8_t id, uint32_t *present_load) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readTemperature(uint8_t id, uint32_t *temperature) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readVoltage(uint8_t id, uint32_t *voltage) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::readHardwareStatus(uint8_t id, uint32_t *hardware_status) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncReadPosition(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncReadVelocity(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncReadLoad(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list) { std::cout << "not implemented" << std::endl; return 0; }
    int StepperDriver::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list) { std::cout << "not implemented" << std::endl; return 0; }


} //DynamixelDriver
