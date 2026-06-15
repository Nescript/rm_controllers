//
// Created by wk on 2026/5/13.
//

#pragma once

#include <memory>
#include <vector>

#include "rm_common/filters/filters.h"
#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/pid_interface.h"

namespace rm_chassis_controllers
{
class Protect : public ModeBase
{
public:
  explicit Protect(PidInterface* left_leg, PidInterface* right_leg, PidInterface* left_theta, PidInterface* right_theta,
                   PidInterface* left_wheel, PidInterface* right_wheel, PidInterface* theta_diff,
                   PidInterface* yaw_vel);
  LogicOutput execute(const LogicInput& input) override;
  const char* name() const override
  {
    return "PROTECT";
  }

private:
  PidInterface *pid_left_leg_, *pid_right_leg_, *pid_left_theta_, *pid_right_theta_, *pid_left_wheel_, *pid_right_wheel_,
      *pid_theta_diff_, *pid_yaw_vel_;
  std::shared_ptr<RampFilter<double>> ramp_length_des_l_, ramp_length_des_r_, ramp_angle_des_l_, ramp_angle_des_r_;
};
}  // namespace rm_chassis_controllers
