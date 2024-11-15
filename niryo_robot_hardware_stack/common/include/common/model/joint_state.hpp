/*
joint_state.hpp
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

#ifndef JOINT_STATE_H
#define JOINT_STATE_H

#include "abstract_motor_state.hpp"
#include "hardware_type_enum.hpp"
#include "component_type_enum.hpp"

#include <cstdint>
#include <string>
#include <limits>
#include <optional>
#include <iostream>

namespace common
{
namespace model
{
/**
 * @brief The JointState class
 */
class JointState : public AbstractMotorState
{
public:
  JointState() = default;
  JointState(std::string name, EHardwareType type, EComponentType component_type, EBusProtocol bus_proto, uint8_t id);

  ~JointState() override = default;

  void setName(std::string& name);
  void setDirection(int8_t direction);
  void setOffsetPosition(double offset_position);
  void setDefaultHomePosition(double default_home_position);
  void setHomePosition(double home_position);
  void setLimitPositionMax(double max_position);
  void setLimitPositionMin(double min_position);
  void setTorquePercentage(uint8_t torque_percentage);

  void resetHomePosition();

  std::string getName() const;
  int8_t getDirection() const;
  double getOffsetPosition() const;
  double getHomePosition() const;
  double getLimitPositionMax() const;
  double getLimitPositionMin() const;
  uint8_t getTorquePercentage() const;
  bool isValidPosition(double position);

  virtual bool operator==(const JointState& other) const;

  virtual int to_motor_pos(double rad_pos) = 0;
  virtual double to_rad_pos(int motor_pos) = 0;

  virtual int to_motor_vel(double rad_vel) = 0;
  virtual double to_rad_vel(int motor_vel) = 0;

  // AbstractMotorState interface
  void reset() override;
  bool isValid() const override;
  std::string str() const override;

public:
  double pos{ 0.0 };
  double cmd{ 0.0 };
  double vel{ 0.0 };
  double eff{ 0.0 };

protected:
  std::string _name;
  int8_t _direction{ 1 };
  double _offset_position{ 0.0 };
  std::optional<double> _home_position;
  double _default_home_position{ 0.0 };
  uint8_t _torque_percentage{ 0 };

  // joint limit used to calibration ned/one and protect joint move out of bound
  double _limit_position_min{ 0.0 };
  double _limit_position_max{ 0.0 };

protected:
  // see
  // https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#c67-a-polymorphic-class-should-suppress-public-copymove
  JointState(const JointState&) = default;
  JointState(JointState&&) = default;

  JointState& operator=(JointState&&) = default;
  JointState& operator=(const JointState&) = default;
};

/**
 * @brief JointState::getName
 * @return
 */
inline std::string JointState::getName() const
{
  return _name;
}

/**
 * @brief JointState::isValidPosition
 * @return
 */
inline bool JointState::isValidPosition(double position)
{
  return (position >= (_limit_position_min - std::numeric_limits<float>::epsilon()) &&
          position <= (_limit_position_max + std::numeric_limits<float>::epsilon()));
}

/**
 * @brief JointState::getOffsetPosition
 * @return
 */
inline double JointState::getOffsetPosition() const
{
  return _offset_position;
}

/**
 * @brief Reset optional custom home position to std::nullopt
 *
 */
inline void JointState::resetHomePosition()
{
  _home_position.reset();
}

/**
 * @brief JointState::getHomePosition
 * @return Custom home position if found, default home position otherwise
 */
inline double JointState::getHomePosition() const
{
  if (!_home_position.has_value())
  {
    return _default_home_position;
  }
  return _home_position.value();
}

/**
 * @brief JointState::getLimitPosition
 * @return
 */
inline double JointState::getLimitPositionMax() const
{
  return _limit_position_max;
}

/**
 * @brief JointState::getLimitPosition
 * @return
 */
inline double JointState::getLimitPositionMin() const
{
  return _limit_position_min;
}

/**
 * @brief JointState::getDirection
 * @return
 */
inline int8_t JointState::getDirection() const
{
  return _direction;
}

/**
 * @brief JointState::getTorquePercentage
 * @return
 */
inline uint8_t JointState::getTorquePercentage() const
{
  return _torque_percentage;
}

}  // namespace model
}  // namespace common

#endif  // JOINT_STATE_H
