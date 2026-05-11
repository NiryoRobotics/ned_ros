#pragma once

#include <chrono>
#include <thread>

namespace common
{

class SteadyRate
{
public:
  explicit SteadyRate(double frequency)
    : _period{ static_cast<long>(1e9 / frequency) }
    , _last{ std::chrono::steady_clock::now() }
  {
  }

  void sleep()
  {
    const auto target = _last + _period;
    const auto now = std::chrono::steady_clock::now();
    if (now < target)
      std::this_thread::sleep_until(target);
    _last = std::chrono::steady_clock::now();
  }

  double expectedCycleTime() const
  {
    return std::chrono::duration<double>(_period).count();
  }

private:
  std::chrono::nanoseconds _period;
  std::chrono::steady_clock::time_point _last;
};

}  // namespace common
