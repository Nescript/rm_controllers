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
class StandUp : public ModeBase
{
  struct StandUpLegCommand
  {
    double desired_length;
    double desired_angle;
    double desired_angle_vel;
  };

public:
  StandUp(PidInterface* pid_left_leg, PidInterface* pid_right_leg, PidInterface* pid_left_theta,
          PidInterface* pid_right_theta, PidInterface* pid_left_theta_vel, PidInterface* pid_right_theta_vel);
  LogicOutput execute(const LogicInput& input) override;
  const char* name() const override
  {
    return "STAND_UP";
  }

private:
  void setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const LegOrientation& other_leg_orientation,
                      const double& leg_length, const double& leg_theta, LegOrientation& leg_orientation,
                      StandUpLegCommand& legCommand, bool& stop_flag, bool& arrive_flag, double& arrive_time,
                      double time, const std::shared_ptr<LegStateThresholdParams>& threshold);
  void detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, LegOrientation& leg_orientation,
                      const std::shared_ptr<LegStateThresholdParams>& threshold);
  inline LegCommand computePidLegCommand(const StandUpLegCommand& leg_command, const VMCPtr& vmc_,
                                         PidInterface& length_pid, PidInterface& angle_pid, PidInterface& angle_vel_pid,
                                         const LegOrientation& leg_orientation, double period,
                                         double feedforward_force = 0.0f);
  PidInterface *pid_left_leg_, *pid_right_leg_, *pid_left_theta_, *pid_right_theta_, *pid_left_theta_vel_,
      *pid_right_theta_vel_;
  LegOrientation left_leg_orientation, right_leg_orientation;
  StandUpLegCommand left_leg_command_, right_leg_command_;
  bool left_stop_{ false }, right_stop_{ false };
  double left_arrive_time_{ 0.0 }, right_arrive_time_{ 0.0 };
  bool left_arrive_flag_{ false }, right_arrive_flag_{ false };
  std::shared_ptr<RampFilter<double>> ramp_length_des_l_, ramp_length_des_r_, ramp_angle_des_l_, ramp_angle_des_r_;
};
}  // namespace rm_chassis_controllers
