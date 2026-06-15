//
// Created by wk on 2025/11/1.
//
#pragma once

#include <memory>
#include <vector>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/pid_interface.h"

namespace rm_chassis_controllers
{
class Upstairs : public ModeBase
{
public:
  Upstairs(PidInterface* pid_left_leg, PidInterface* pid_right_leg, PidInterface* pid_left_theta,
           PidInterface* pid_right_theta, PidInterface* pid_left_theta_vel, PidInterface* pid_right_theta_vel);
  LogicOutput execute(const LogicInput& input) override;
  const char* name() const override
  {
    return "UPSTAIRS";
  }

private:
  void detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, LegOrientation& leg_state,
                      const std::shared_ptr<LegStateThresholdParams>& threshold);
  inline LegCommand computePidLegCommand(double desired_length, double desired_angle, const VMCPtr& vmc_,
                                         PidInterface& length_pid, PidInterface& angle_pid, PidInterface& angle_vel_pid,
                                         const LegOrientation& leg_state, double period, double feedforward_force);

  PidInterface *pid_left_leg_, *pid_right_leg_, *pid_left_theta_, *pid_right_theta_, *pid_left_theta_vel_,
      *pid_right_theta_vel_;
  LegOrientation left_leg_orientation, right_leg_orientation;
};
}  // namespace rm_chassis_controllers
