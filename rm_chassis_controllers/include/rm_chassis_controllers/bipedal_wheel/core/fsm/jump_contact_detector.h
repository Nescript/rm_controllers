#pragma once

#include <cmath>

namespace bipedal_wheel_core
{
// Requires a confirmed flight before accepting contact from either wheel.
class JumpContactDetector
{
public:
  void reset()
  {
    airborne_ = false;
    airborne_duration_ = left_contact_duration_ = right_contact_duration_ = 0.0;
  }

  bool update(double left_force, double right_force, double unstick_threshold, double dt)
  {
    if (!(dt > 0.0) || !std::isfinite(dt) || !std::isfinite(left_force) || !std::isfinite(right_force))
    {
      airborne_duration_ = left_contact_duration_ = right_contact_duration_ = 0.0;
      return false;
    }
    if (!airborne_)
    {
      airborne_duration_ = left_force < unstick_threshold && right_force < unstick_threshold ?
                               airborne_duration_ + dt : 0.0;
      airborne_ = airborne_duration_ >= 0.01;
      return false;
    }
    // A 5 N margin and 10 ms continuous contact reject threshold chatter/spikes.
    const double contact_threshold = unstick_threshold + 5.0;
    left_contact_duration_ = left_force >= contact_threshold ? left_contact_duration_ + dt : 0.0;
    right_contact_duration_ = right_force >= contact_threshold ? right_contact_duration_ + dt : 0.0;
    return left_contact_duration_ >= 0.01 || right_contact_duration_ >= 0.01;
  }

  bool isAirborne() const { return airborne_; }

private:
  bool airborne_{ false };
  double airborne_duration_{ 0.0 }, left_contact_duration_{ 0.0 }, right_contact_duration_{ 0.0 };
};
}  // namespace bipedal_wheel_core

