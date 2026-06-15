//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <memory>
#include <vector>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/pid_interface.h"

namespace rm_chassis_controllers
{
class Recover : public ModeBase
{
  enum RecoveryChassisState
  {
    ForwardSlip,
    BackwardSlip,
  };

  enum LegRecoveryCalibratedState
  {
    INITIALIZED,
    START,
    MOVING,
    MOVING_TOGETHER,
    STOP,
  };

  enum LegRecoveryState
  {
    NotReady,
    Ready
  };

public:
  Recover(PidInterface* left_leg, PidInterface* right_leg, PidInterface* left_theta, PidInterface* right_theta,
          PidInterface* theta_diff);
  LogicOutput execute(const LogicInput& input) override;
  const char* name() const override
  {
    return "RECOVER";
  }

private:
  inline void detectChassisStateToRecover(const ChassisState& chassis_state);
  inline void detectLegRecoveryState(LegRecoveryState& recovery_state, const double& leg_pos,
                                     const std::shared_ptr<LegStateThresholdParams>& threshold);
  PidInterface *pid_left_leg_, *pid_right_leg_, *pid_left_theta_, *pid_right_theta_, *pid_theta_diff_;
  double leg_recovery_velocity_{ 5.0 }, threshold_{ 0.05 }, leg_theta_diff_{ 0.0 }, desired_leg_length_{ 0.36 };
  double left_leg_recovery_feed_forward{ 10.0 }, right_leg_recovery_feed_forward{ 10.0 };
  const double leg_recovery_velocity_const_{ 5.0 };
  LegRecoveryState left_recovery_leg, right_recovery_leg;
  RecoveryChassisState recovery_chassis_state_{ ForwardSlip };
  bool detectd_flag{ false };
};
}  // namespace rm_chassis_controllers
