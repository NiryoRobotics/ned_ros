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

namespace ttl_driver
{

/**
 * @brief The EndEffectorDriver class
 */
template<typename reg_type>
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

    public:
        int setFreeDriveButtonConfiguration(uint8_t id, uint32_t configuration);
        int setSaveButtonConfiguration(uint8_t id, uint32_t configuration);
        int setCustomButtonConfiguration(uint8_t id, uint32_t configuration);

        int readFreeDriveButtonConfiguration(uint8_t id, uint32_t& configuration);
        int readSaveButtonConfiguration(uint8_t id, uint32_t& configuration);
        int readCustomButtonConfiguration(uint8_t id, uint32_t& configuration);

        int readFreeDriveButtonStatus(uint8_t id, uint32_t& status);
        int readSaveButtonStatus(uint8_t id, uint32_t& status);
        int readCustomButtonStatus(uint8_t id, uint32_t& status);

        int readAccelerometerXValue(uint8_t id, uint32_t& x_value);
        int readAccelerometerYValue(uint8_t id, uint32_t& y_value);
        int readAccelerometerZValue(uint8_t id, uint32_t& z_value);

        int readCollisionStatus(uint8_t id, bool&status);

        int readDigitalInput(uint8_t id, bool& in);
        int setDigitalOutput(uint8_t id, bool out);
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

template<typename reg_type>
EndEffectorDriver<reg_type>::~EndEffectorDriver()
{
}


//*****************************
// AbstractTtlDriver interface
//*****************************

template<typename reg_type>
std::string EndEffectorDriver<reg_type>::str() const
{
    return common::model::MotorTypeEnum(reg_type::motor_type).toString() + " : " + AbstractTtlDriver::str();
}

template<typename reg_type>
std::string EndEffectorDriver<reg_type>::interpreteErrorState(uint32_t /*hw_state*/)
{
    return "no error table";
}

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

template<typename reg_type>
int EndEffectorDriver<reg_type>::readFirmwareVersion(uint8_t id, uint32_t &version)
{
    return read(reg_type::ADDR_FIRMWARE_VERSION, reg_type::SIZE_FIRMWARE_VERSION, id, version);
}

// ram read

template<typename reg_type>
int EndEffectorDriver<reg_type>::readTemperature(uint8_t id, uint32_t& temperature)
{
    return read(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id, temperature);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::readVoltage(uint8_t id, uint32_t& voltage)
{
    return read(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id, voltage);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::readHwErrorStatus(uint8_t id, uint32_t& hardware_status)
{
    hardware_status = 0;
    std::cout << "readHwErrorStatus not yet implemented" << std::endl;
    //return read(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id, hardware_status);
    return COMM_RX_FAIL;
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadTemperature(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    return syncRead(reg_type::ADDR_PRESENT_TEMPERATURE, reg_type::SIZE_PRESENT_TEMPERATURE, id_list, temperature_list);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadVoltage(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)
{
    return syncRead(reg_type::ADDR_PRESENT_VOLTAGE, reg_type::SIZE_PRESENT_VOLTAGE, id_list, voltage_list);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::syncReadHwErrorStatus(const std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    std::cout << "readHwErrorStatus not yet implemented" << std::endl;

   // return syncRead(reg_type::ADDR_HW_ERROR_STATUS, reg_type::SIZE_HW_ERROR_STATUS, id_list, hw_error_list);
    return COMM_RX_FAIL;
}

// buttons configuration

template<typename reg_type>
int EndEffectorDriver<reg_type>::setFreeDriveButtonConfiguration(uint8_t id, uint32_t configuration)
{
    return write(reg_type::ADDR_FREE_DRIVE_CONFIG, reg_type::SIZE_FREE_DRIVE_CONFIG, id, configuration);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::setSaveButtonConfiguration(uint8_t id, uint32_t configuration)
{
    return write(reg_type::ADDR_SAVE_POSITION_CONFIG, reg_type::SIZE_SAVE_POSITION_CONFIG, id, configuration);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::setCustomButtonConfiguration(uint8_t id, uint32_t configuration)
{
    return write(reg_type::ADDR_CUSTOM_BUTTON_CONFIG, reg_type::SIZE_CUSTOM_BUTTON_CONFIG, id, configuration);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::readFreeDriveButtonConfiguration(uint8_t id, uint32_t& configuration)
{
    return read(reg_type::ADDR_FREE_DRIVE_CONFIG, reg_type::SIZE_FREE_DRIVE_CONFIG, id, configuration);
}
    
template<typename reg_type>
int EndEffectorDriver<reg_type>::readSaveButtonConfiguration(uint8_t id, uint32_t& configuration)
{
    return read(reg_type::ADDR_SAVE_POSITION_CONFIG, reg_type::SIZE_SAVE_POSITION_CONFIG, id, configuration);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::readCustomButtonConfiguration(uint8_t id, uint32_t& configuration)
{
    return read(reg_type::ADDR_CUSTOM_BUTTON_CONFIG, reg_type::SIZE_CUSTOM_BUTTON_CONFIG, id, configuration);
}

// buttons status

template<typename reg_type>
int EndEffectorDriver<reg_type>::readFreeDriveButtonStatus(uint8_t id, uint32_t& status)
{
    return read(reg_type::ADDR_FREE_DRIVE_STATUS, reg_type::SIZE_FREE_DRIVE_STATUS, id, status);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::readSaveButtonStatus(uint8_t id, uint32_t& status)
{
    return read(reg_type::ADDR_SAVE_POSITION_STATUS, reg_type::SIZE_SAVE_POSITION_STATUS, id, status);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::readCustomButtonStatus(uint8_t id, uint32_t& status)
{
    return read(reg_type::ADDR_CUSTOM_BUTTON_STATUS, reg_type::SIZE_CUSTOM_BUTTON_STATUS, id, status);
}

// accelerometers and collision

template<typename reg_type>
int EndEffectorDriver<reg_type>::readAccelerometerXValue(uint8_t id, uint32_t& x_value)
{
    return read(reg_type::ADDR_ACCELERO_VALUE_X, reg_type::SIZE_ACCELERO_VALUE_X, id, x_value);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::readAccelerometerYValue(uint8_t id, uint32_t& y_value)
{
    return read(reg_type::ADDR_ACCELERO_VALUE_Y, reg_type::SIZE_ACCELERO_VALUE_Y, id, y_value);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::readAccelerometerZValue(uint8_t id, uint32_t& z_value)
{
    return read(reg_type::ADDR_ACCELERO_VALUE_Z, reg_type::SIZE_ACCELERO_VALUE_Z, id, z_value);
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::readCollisionStatus(uint8_t id, bool& status)
{
    status = false;
    ROS_INFO("EndEffectorDriver<reg_type>::readCollisionStatus: need to be implemented!");
    return 0;
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::readDigitalInput(uint8_t id, bool& in)
{
    uint32_t value;
    int res = read(reg_type::ADDR_DIGITAL_IN, reg_type::SIZE_DIGITAL_IN, id, value);
    in = (value > 0) ? true : false;
    return res;
}

template<typename reg_type>
int EndEffectorDriver<reg_type>::setDigitalOutput(uint8_t id, bool out)
{
    return read(reg_type::ADDR_DIGITAL_OUT, reg_type::SIZE_DIGITAL_OUT, id, (out > 0) ? 1 : 0);
}

} // ttl_driver

#endif // EndEffectorDriver
