#pragma once

#include "bipedal_wheel/core/fsm/fsm_context.h"

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class Upstairs
{
public:
  Upstairs() = default;

  void execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx);

private:
  void detectLegState(double theta, const LegStateThresholdParams& threshold, LegOrientation& leg_state,
                      LoggerType& logger);

  LegCommand computePidLegCommand(double desired_length, double desired_angle, VMC* vmc_, PidType& length_pid,
                                  PidType& angle_pid, PidType& angle_vel_pid, const LegOrientation& leg_orientation,
                                  double dt, double feedforward_force);

  LegOrientation left_leg_orientation = LegOrientation::UNDER;
  LegOrientation right_leg_orientation = LegOrientation::UNDER;
};

}  // namespace bipedal_wheel_core
