#pragma once

#include "bipedal_wheel/core/fsm/fsm_context.h"
#include <memory>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class StandUp
{
  struct StandUpLegCommand
  {
    double desired_length = 0.0;
    double desired_angle = 0.0;
    double desired_angle_vel = 0.0;
  };

public:
  StandUp();

  void execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx);

private:
  void setUpLegMotion(double theta, double leg_length, double leg_theta, const LegStateThresholdParams& threshold,
                      const LegOrientation& other_leg_orientation, LegOrientation& leg_orientation,
                      StandUpLegCommand& legCommand, bool& stop_flag, bool& arrive_flag, double& arrive_time_counter,
                      double dt);

  void detectLegState(double theta, const LegStateThresholdParams& threshold, LegOrientation& leg_orientation,
                      LoggerType& logger);

  LegCommand computePidLegCommand(const StandUpLegCommand& leg_command, VMC* vmc_, PidType& length_pid,
                                  PidType& angle_pid, PidType& angle_vel_pid, const LegOrientation& leg_orientation,
                                  double dt, double feedforward_force);

  std::shared_ptr<RampFilterType> ramp_length_des_l_;
  std::shared_ptr<RampFilterType> ramp_length_des_r_;
  std::shared_ptr<RampFilterType> ramp_angle_des_l_;
  std::shared_ptr<RampFilterType> ramp_angle_des_r_;
  LegOrientation left_leg_orientation = LegOrientation::UNDER;
  LegOrientation right_leg_orientation = LegOrientation::UNDER;
  double theta_des_l = 0.0, theta_des_r = 0.0, length_des_l = 0.12, length_des_r = 0.12;
  StandUpLegCommand left_leg_command_{};
  StandUpLegCommand right_leg_command_{};
  bool left_stop_ = false, right_stop_ = false;
  double left_arrive_time_ = 0.0, right_arrive_time_ = 0.0;
  bool left_arrive_flag_ = false, right_arrive_flag_ = false;
};

}  // namespace bipedal_wheel_core
