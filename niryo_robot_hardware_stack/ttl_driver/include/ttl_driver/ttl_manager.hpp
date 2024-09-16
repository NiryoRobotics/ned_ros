/*
ttl_manager.hpp
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

#ifndef TTL_DRIVER_HPP
#define TTL_DRIVER_HPP

#include "common/util/util_defs.hpp"
#include "common/util/i_bus_manager.hpp"

// cpp
#include <memory>
#include <ros/ros.h>
#include <string>
#include <thread>
#include <queue>
#include <functional>
#include <algorithm>
#include <set>
#include <utility>

// ros
#include "ros/node_handle.h"

// niryo
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "niryo_robot_msgs/MotorHeader.h"
#include "niryo_robot_msgs/SetInt.h"
#include "niryo_robot_msgs/CommandStatus.h"

#include "ttl_driver/abstract_motor_driver.hpp"
#include "ttl_driver/abstract_stepper_driver.hpp"
#include "ttl_driver/fake_ttl_data.hpp"
#include "ttl_driver/MotorCommand.h"

#include "common/model/dxl_motor_state.hpp"
#include "common/model/synchronize_motor_cmd.hpp"
#include "common/model/single_motor_cmd.hpp"
#include "common/model/stepper_calibration_status_enum.hpp"
#include "common/model/conveyor_state.hpp"


namespace ttl_driver
{

/**
 * Parameters for DXL
*/
constexpr float TTL_BUS_PROTOCOL_VERSION = 2.0;
constexpr int TTL_FAIL_OPEN_PORT         = -4500;

constexpr int TTL_FAIL_PORT_SET_BAUDRATE = -4501;
constexpr int TTL_FAIL_SETUP_GPIO        = -4502;

constexpr int TTL_SCAN_OK                = 0;
constexpr int TTL_SCAN_MISSING_MOTOR     = -50;
constexpr int TTL_SCAN_UNALLOWED_MOTOR   = -51;
constexpr int TTL_WRONG_TYPE             = -52;

/**
 * Parameters for Stepper
*/

/**
 * @brief The TtlManager class manages the different motor drivers connected on the TTL bus
 * it is used by ttl_interface_core to send or receive data to the ttl bus
 * it also manages the lifecycle of all motors driver (do we need to add also the end effector driver in it ?)
 *
 */
class TtlManager : public common::util::IBusManager
{
public:
    TtlManager() = delete;
    TtlManager( ros::NodeHandle& nh );
    ~TtlManager() override;
    // see https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c21-if-you-define-or-delete-any-copy-move-or-destructor-function-define-or-delete-them-all
    TtlManager( const TtlManager& ) = delete;
    TtlManager( TtlManager&& ) = delete;
    TtlManager& operator= ( TtlManager && ) = delete;
    TtlManager& operator= ( const TtlManager& ) = delete;

    // IBusManager Interface
    bool init(ros::NodeHandle& nh) override;

    int addHardwareComponent(std::shared_ptr<common::model::AbstractHardwareState> &&state) override;

    void removeHardwareComponent(uint8_t id) override;
    bool isConnectionOk() const override;

    int scanAndCheck() override;
    bool ping(uint8_t id) override;

    size_t getNbMotors() const override;
    void getBusState(bool& connection_state, std::vector<uint8_t>& motor_id, std::string& debug_msg) const override;
    std::string getErrorMessage() const override;

    // commands

    int changeId(common::model::EHardwareType motor_type, uint8_t old_id, uint8_t new_id);

    int writeSynchronizeCommand(std::unique_ptr<common::model::AbstractTtlSynchronizeMotorCmd >&& cmd);
    int writeSingleCommand(std::unique_ptr<common::model::AbstractTtlSingleMotorCmd >&& cmd);

    void executeJointTrajectoryCmd(std::vector<std::pair<uint8_t, uint32_t> > cmd_vec);

    int rebootHardware(uint8_t id);

    int setLeds(int led);

    int sendCustomCommand(uint8_t id, int reg_address, int value, int byte_number);
    int readCustomCommand(uint8_t id, int32_t reg_address, int &value, int byte_number);

    void resetTorques();

    // read status
    bool readHardwareStatus();
    bool readEndEffectorStatus();
    uint8_t readSteppersStatus();
    uint8_t readNed3ProSteppersStatus();
    bool readJointsStatus();
    bool readHomingAbsPosition();
    bool readCollisionStatus();

    int readMotorPID(uint8_t id,
                     uint16_t& pos_p_gain, uint16_t& pos_i_gain, uint16_t& pos_d_gain,
                     uint16_t& vel_p_gain, uint16_t& vel_i_gain,
                     uint16_t& ff1_gain, uint16_t& ff2_gain);

    int readVelocityProfile(uint8_t id,
                            uint32_t& v_start, uint32_t& a_1, uint32_t& v_1,
                            uint32_t& a_max, uint32_t& v_max, uint32_t& d_max,
                            uint32_t& d_1, uint32_t& v_stop);

    int readControlMode(uint8_t id, uint8_t& control_mode);

    int readMoving(uint8_t id, uint8_t &status);

    //calibration
    void startCalibration() override;
    void resetCalibration() override;

    int32_t getCalibrationResult(uint8_t id) const override;
    common::model::EStepperCalibrationStatus getCalibrationStatus() const override;
    bool needCalibration() const;
    bool isCalibrationInProgress() const;

    // getters
    int getAllIdsOnBus(std::vector<uint8_t> &id_list);

    uint32_t getPosition(const common::model::JointState &motor_state);
    int getLedState() const;

    std::vector<std::shared_ptr<common::model::JointState>> getMotorsStates() const;
    std::vector<std::shared_ptr<common::model::ConveyorState>> getConveyorsStates() const;
    std::shared_ptr<common::model::AbstractHardwareState> getHardwareState(uint8_t motor_id) const;

    std::vector<uint8_t> getRemovedMotorList() const override;

    bool getCollisionStatus() const;

    bool hasEndEffector() const;

private:
    // IBusManager Interface
    int setupCommunication() override;
    void addHardwareDriver(common::model::EHardwareType hardware_type) override;

    // Config params using in fake driver
    void readFakeConfig(bool use_simu_gripper, bool use_simu_conveyor);
    template<typename Reg>
    void retrieveFakeMotorData(const std::string& current_ns, std::map<uint8_t, Reg>& fake_params);

    // check if hardware is a motor or not
    // this helps get only one driver to use for all motors to get/set on the same address
    bool isMotorType(common::model::EHardwareType type);

    bool checkCollision();

    struct GetJointsStepperDriverResult {
        std::shared_ptr<ttl_driver::AbstractStepperDriver> driver;
        common::model::EHardwareType hardware_type;
    };
    GetJointsStepperDriverResult getJointsStepperDriver();

private:
    ros::NodeHandle _nh;
    std::shared_ptr<dynamixel::PortHandler> _portHandler;
    std::shared_ptr<dynamixel::PacketHandler> _packetHandler;

    mutable std::mutex _sync_mutex;

    std::string _device_name;
    int _baudrate{1000000};

    std::vector<uint8_t> _all_ids_connected; // with all ttl motors connected (including the tool)
    std::vector<uint8_t> _removed_motor_id_list;

    // state of a component for a given id
    std::map<uint8_t, std::shared_ptr<common::model::AbstractHardwareState> > _state_map;
    // map of associated ids for a given hardware type
    std::map<common::model::EHardwareType, std::vector<uint8_t> > _ids_map;
    // map of drivers for a given hardware type (dxl, stepper, end effector)
    std::map<common::model::EHardwareType, std::shared_ptr<ttl_driver::AbstractTtlDriver> > _driver_map;

    // default ttl driver is always available
    std::shared_ptr<ttl_driver::AbstractTtlDriver> _default_ttl_driver;

    // vector of ids of motors and conveyors
    // Theses vector help remove loop not necessary
    std::vector<uint8_t> _conveyor_list;

    // for hardware control
    bool _is_connection_ok{false};
    std::string _debug_error_message;

    uint32_t _hw_fail_counter_read{0};
    uint32_t _end_effector_fail_counter_read{0};

    int _led_state = 0;
    std::string _led_motor_type_cfg;

    static constexpr uint32_t MAX_HW_FAILURE = 150;
    static constexpr uint32_t MAX_READ_EE_FAILURE = 150;

    // at init, no hw, so no calib needed
    common::model::EStepperCalibrationStatus _calibration_status{common::model::EStepperCalibrationStatus::OK};

    // for simulation only
    std::shared_ptr<FakeTtlData> _fake_data;
    bool _simulation_mode{false};

    // status to track collision status
    bool _collision_status{false};
    double _last_collision_detection_activating{0.0};
    bool _isRealCollision{true};
    bool _isWrongAction{false};

    ros::Publisher _calibration_status_publisher;

    std::vector<uint32_t> _position_list;
    std::vector<uint8_t> _position_goal_ids;
    std::vector<uint32_t> _position_goal_params;

    class CalibrationMachineState
    {

    public:
        enum class State
        {
          IDLE = 1,
          STARTING = 2,
          IN_PROGRESS = 3,
          UPDATING = 4
        };

        void reset()
        {
            s = State::IDLE;
        }

        void start()
        {
            s = State::STARTING;
            _time = ros::Time::now().toSec(); // keep last date updated
        }

        /**
         * @brief next : next state, stops at updating (dont circle)
         */
        void next()
        {
            int newState = static_cast<int>(s);
            if (State::UPDATING != s)
               newState++;

            _time = ros::Time::now().toSec(); // keep last date updated
            s = static_cast<State>(newState);
        }

        State status()
        {
            return s;
        }

        bool isTimeout()
        {
            return (ros::Time::now().toSec() - _time > _calibration_timeout);
        }

    private:
        State s{State::UPDATING};
        double _time{};
        static constexpr double _calibration_timeout{5.0};
    };

    CalibrationMachineState _calib_machine_state;

};

// inline getters

/**
 * @brief TtlManager::isConnectionOk
 * @return
 */
inline
bool TtlManager::isConnectionOk() const
{
    return _is_connection_ok;
}

/**
 * @brief TtlManager::getNbMotors
 * @return
 */
inline
size_t TtlManager::getNbMotors() const
{
    return _state_map.size();
}

/**
 * @brief TtlManager::getRemovedMotorList
 * @return
 */
inline
std::vector<uint8_t> TtlManager::getRemovedMotorList() const
{
    return _removed_motor_id_list;
}

/**
 * @brief TtlManager::getErrorMessage
 * @return
 */
inline
std::string TtlManager::getErrorMessage() const
{
    return _debug_error_message;
}

/**
 * @brief TtlManager::getCalibrationStatus
 * @return
 */
inline
common::model::EStepperCalibrationStatus
TtlManager::getCalibrationStatus() const
{
  return _calibration_status;
}

/**
 * @brief TtlManager::needCalibration
 * @return
 */
inline
bool TtlManager::needCalibration() const
{
  return (common::model::EStepperCalibrationStatus::OK != getCalibrationStatus() &&
          common::model::EStepperCalibrationStatus::IN_PROGRESS != getCalibrationStatus());
}

/**
 * @brief TtlManager::isCalibrationInProgress
 * @return
 */
inline
bool TtlManager::isCalibrationInProgress() const
{
    return (common::model::EStepperCalibrationStatus::IN_PROGRESS == getCalibrationStatus());
}

/**
 * @brief TtlManager::hasEndEffector
 * @return
 */
inline
bool TtlManager::hasEndEffector() const
{
    return (_driver_map.count(common::model::EHardwareType::END_EFFECTOR) || _driver_map.count(common::model::EHardwareType::NED3PRO_END_EFFECTOR) ||
            _driver_map.count(common::model::EHardwareType::FAKE_END_EFFECTOR));
}

/**
 * @brief TtlManager::retrieveFakeMotorData
 * @param current_ns
 * @param fake_params
 */
template<typename Reg>
void TtlManager::retrieveFakeMotorData(const std::string& current_ns, std::map<uint8_t, Reg> &fake_params)
{
    std::vector<int> hw_ids;
    _nh.getParam(current_ns + "id", hw_ids);

    std::vector<int> hw_positions;
    _nh.getParam(current_ns + "position", hw_positions);
    assert(hw_ids.size() == hw_positions.size());

    std::vector<int> hw_velocities;
    _nh.getParam(current_ns + "velocity", hw_velocities);
    assert(hw_ids.size() == hw_velocities.size());

    std::vector<int> hw_temperatures;
    _nh.getParam(current_ns + "temperature", hw_temperatures);
    assert(hw_positions.size() == hw_temperatures.size());

     std::vector<double> hw_voltages;
    _nh.getParam(current_ns + "voltage", hw_voltages);
    assert(hw_temperatures.size() == hw_voltages.size());

    std::vector<int> hw_min_positions;
    _nh.getParam(current_ns + "min_position", hw_min_positions);
    assert(hw_voltages.size() == hw_min_positions.size());

    std::vector<int> hw_max_positions;
    _nh.getParam(current_ns + "max_position", hw_max_positions);
    assert(hw_min_positions.size() == hw_max_positions.size());

    std::vector<int> hw_model_numbers;
    _nh.getParam(current_ns + "model_number", hw_model_numbers);
    assert(hw_max_positions.size() == hw_model_numbers.size());

     std::vector<std::string> hw_firmwares;
    _nh.getParam(current_ns + "firmware", hw_firmwares);
    assert(hw_firmwares.size() == hw_firmwares.size());

    for (size_t i = 0; i < hw_ids.size(); i++)
    {
        Reg tmp;
        tmp.id = static_cast<uint8_t>(hw_ids.at(i));
        tmp.position = static_cast<uint32_t>(hw_positions.at(i));
        tmp.velocity = static_cast<uint32_t>(hw_velocities.at(i));
        tmp.temperature = static_cast<uint8_t>(hw_temperatures.at(i));
        tmp.voltage = hw_voltages.at(i);
        tmp.model_number = static_cast<uint16_t>(hw_model_numbers.at(i));
        tmp.firmware = hw_firmwares.at(i);
        fake_params.insert(std::make_pair(tmp.id, tmp));
    }
}

/**
 * @brief TtlManager::getCollisionStatus
 * @return
 */
inline
bool TtlManager::getCollisionStatus() const
{
    return _collision_status;
}

} // ttl_driver

#endif // TTLDRIVER_HPP
