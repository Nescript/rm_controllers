#pragma once

#include "bipedal_wheel/core/core_types.h"
#include <algorithm>

namespace bipedal_wheel_core
{
// Keep full feedback during push-off. In OFF_GROUND, use the calibrated
// world-frame leg angle target for the air-only leg torque feedback.
inline void maskJumpFeedback(Eigen::Matrix<double, STATE_DIM, 1>& error, JumpPhase phase,
                             double world_leg_angle, double world_leg_angular_velocity, double theta_bias)
{
  if (phase != JumpPhase::OFF_GROUND)
    return;

  error(POS) = 0.0;
  error(VEL) = 0.0;
  error(THETA) = std::clamp(world_leg_angle - theta_bias, -0.6, 0.6);
  error(D_THETA) = world_leg_angular_velocity;
  error(PITCH) = 0.0;
  error(D_PITCH) = 0.0;
}
}  // namespace bipedal_wheel_core
