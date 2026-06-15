//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <memory>
#include <vector>

#include "rm_common/filters/filters.h"
#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/pid_interface.h"

namespace rm_chassis_controllers
{
class Normal : public ModeBase
{
public:
  Normal(PidInterface* pid_left_leg, PidInterface* pid_right_leg, PidInterface* pid_yaw_vel,
         PidInterface* pid_theta_diff, PidInterface* pid_roll, PidInterface* pid_wheel_vel_diff);
  LogicOutput execute(const LogicInput& input) override;
  const char* name() const override
  {
    return "NORMAL";
  }

private:
  double calculateSupportForce(double F, double Tp, double leg_length, const double& leg_len_spd, double acc_z,
                               Eigen::Matrix<double, STATE_DIM, 1> x, const std::shared_ptr<ModelParams>& model_params,
                               double period);
  bool unstickDetection(const double& F_leg, const double& Tp, const double& leg_len_spd, const double& leg_length,
                        const double& acc_z, const std::shared_ptr<ModelParams>& model_params,
                        Eigen::Matrix<double, STATE_DIM, 1> x,
                        const std::shared_ptr<MovingAverageFilter<double>>& supportForceAveragePtr, double period,
                        double time);
  PidInterface *pid_left_leg_, *pid_right_leg_, *pid_yaw_vel_, *pid_theta_diff_, *pid_roll_, *pid_wheel_vel_diff_;
  double lastJumpTime_{ 0.0 };

  double pos_des_{ 0.0 }, leg_length_des{ 0.2 };
  double unstick_threshold{ 15.0 };
  int jump_phase_ = JumpPhase::IDLE, jumpTime_{ 0 };
  bool x_offset_flag_{ false }, protect_flag_{ false };
  std::shared_ptr<MovingAverageFilter<double>> leftSupportForceAveragePtr_, rightSupportForceAveragePtr_;
};
}  // namespace rm_chassis_controllers
