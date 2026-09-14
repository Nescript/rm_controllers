#pragma once

#include "bipedal_wheel/core/fsm/fsm_context.h"

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class Recover
{
  enum RecoveryChassisState
  {
    ForwardSlip,
    BackwardSlip
  };

  enum LegRecoveryState
  {
    NotReady,
    Ready
  };

public:
  Recover() = default;

  void execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx);

private:
  void detectChassisStateToRecover(double pitch, LoggerType& logger);

  void detectLegRecoveryState(LegRecoveryState& leg_recovery_state, double leg_pos, LoggerType& logger);

  double leg_recovery_velocity_ = 5.0;
  double threshold_ = 0.05;
  double leg_theta_diff_ = 0.0;
  double desired_leg_length_ = 0.36;
  double left_leg_recovery_feed_forward_ = 10.0;
  double right_leg_recovery_feed_forward_ = 10.0;
  static constexpr double leg_recovery_velocity_const_ = 5.0;
  LegRecoveryState left_recovery_leg_ = NotReady;
  LegRecoveryState right_recovery_leg_ = NotReady;
  RecoveryChassisState recovery_chassis_state_ = ForwardSlip;
  bool detectd_flag_ = false;
};

}  // namespace bipedal_wheel_core
