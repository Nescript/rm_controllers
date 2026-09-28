#pragma once

#include "bipedal_wheel/core/core_types.h"

namespace bipedal_wheel_core
{
// Translation cannot be corrected by leg actuation during a jump. On the
// ground, keep pitch/leg feedback for takeoff; after confirmed flight, only
// virtual leg angle and angular velocity are fed back to the leg torque.
inline void maskJumpFeedback(Eigen::Matrix<double, STATE_DIM, 1>& error, JumpPhase phase, bool airborne)
{
  if (phase == JumpPhase::IDLE)
    return;

  error(POS) = 0.0;
  error(VEL) = 0.0;
  if (airborne)
  {
    error(PITCH) = 0.0;
    error(D_PITCH) = 0.0;
  }
}
}  // namespace bipedal_wheel_core
