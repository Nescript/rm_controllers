#pragma once

#include "bipedal_wheel/core/core_types.h"
#include <algorithm>

namespace bipedal_wheel_core
{
// Keep the calibrated world-frame leg angle target through flight, as during
// the grounded jump phases.
inline void maskJumpFeedback(Eigen::Matrix<double, STATE_DIM, 1>& error, JumpPhase phase, bool flight_control,
                             double world_leg_angle, double world_leg_angular_velocity, double theta_bias)
{
  if (phase == JumpPhase::IDLE)
    return;

  error(POS) = 0.0;
  error(VEL) = 0.0;
  if (flight_control)
  {
    error(THETA) = std::clamp(world_leg_angle - theta_bias, -0.6, 0.6);
    error(D_THETA) = world_leg_angular_velocity;
    error(PITCH) = 0.0;
    error(D_PITCH) = 0.0;
  }
}
}  // namespace bipedal_wheel_core
