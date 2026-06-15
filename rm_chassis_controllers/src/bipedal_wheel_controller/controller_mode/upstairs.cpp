//
// Created by wk on 2025/11/1.
//

#include "bipedal_wheel_controller/controller_mode/upstairs.h"
#include "bipedal_wheel_controller/math_helpers.h"

namespace rm_chassis_controllers
{
Upstairs::Upstairs(PidInterface* pid_left_leg, PidInterface* pid_right_leg, PidInterface* pid_left_theta,
                   PidInterface* pid_right_theta, PidInterface* pid_left_theta_vel, PidInterface* pid_right_theta_vel)
  : pid_left_leg_(pid_left_leg)
  , pid_right_leg_(pid_right_leg)
  , pid_left_theta_(pid_left_theta)
  , pid_right_theta_(pid_right_theta)
  , pid_left_theta_vel_(pid_left_theta_vel)
  , pid_right_theta_vel_(pid_right_theta_vel)
{
}

LogicOutput Upstairs::execute(const LogicInput& input)
{
  LogicOutput output;
  const auto& left_leg_state = input.leg_state[LEFT];
  const auto& right_leg_state = input.leg_state[RIGHT];
  if (!input.state_changed)
  {
    output.state_changed_requested = true;
    output.complete_stand_requested = false;
    detectLegState(left_leg_state.x, left_leg_orientation, input.leg_threshold_params);
    detectLegState(right_leg_state.x, right_leg_orientation, input.leg_threshold_params);
  }

  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();
  const auto& leg_state_threshold_ = input.leg_threshold_params;

  double theta_des_l{ 1.57 }, theta_des_r{ 1.57 }, length_des_l{ 0.18 }, length_des_r{ 0.18 };
  auto model_params_ = input.model_params;
  auto spring_params_ = input.spring_params;
  double left_spring_force = -f_spring_force(left_pos.L0, model_params_->l, model_params_->l,
                                            spring_params_->f_spring, spring_params_->s2, spring_params_->s3, spring_params_->alpha_s);
  double right_spring_force = -f_spring_force(right_pos.L0, model_params_->l, model_params_->l,
                                             spring_params_->f_spring, spring_params_->s2, spring_params_->s3, spring_params_->alpha_s);

  length_des_l = length_des_r = leg_state_threshold_->upstair_des_length;
  theta_des_l = theta_des_r = leg_state_threshold_->upstair_des_theta;
  
  output.leg_cmd[LEFT] = computePidLegCommand(length_des_l, theta_des_l, left_leg_state.vmc, *pid_left_leg_, *pid_left_theta_,
                                            *pid_left_theta_vel_, left_leg_orientation, input.period, left_spring_force);
  output.leg_cmd[RIGHT] = computePidLegCommand(length_des_r, theta_des_r, right_leg_state.vmc, *pid_right_leg_, *pid_right_theta_,
                                             *pid_right_theta_vel_, right_leg_orientation, input.period, right_spring_force);
  output.wheel_cmd[LEFT] = output.wheel_cmd[RIGHT] = 0.0;

  // Exit
  if (left_pos.theta > leg_state_threshold_->upstair_exit_theta_threshold &&
      right_pos.theta > leg_state_threshold_->upstair_exit_theta_threshold &&
      left_pos.L0 < leg_state_threshold_->upstair_exit_length_threshold &&
      right_pos.L0 < leg_state_threshold_->upstair_exit_length_threshold)
  {
    // Need to handle pubLegLenStatus(true)
    output.next_mode = BalanceMode::STAND_UP;
    output.state_changed_requested = false;
  }
  
  return output;
}

void Upstairs::detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, LegOrientation& leg_state,
                               const std::shared_ptr<LegStateThresholdParams>& threshold)
{
  if (!threshold)
  {
    return;
  }
  if (x[0] > threshold->under_lower && x[0] < threshold->under_upper)
    leg_state = LegOrientation::UNDER;
  else if ((x[0] < threshold->front_lower && x[0] > -M_PI) ||
           (x[0] < M_PI && x[0] > threshold->front_upper))
    leg_state = LegOrientation::FRONT;
  else if (x[0] > threshold->behind_lower && x[0] < threshold->behind_upper)
    leg_state = LegOrientation::BEHIND;
}

LegCommand Upstairs::computePidLegCommand(double desired_length, double desired_angle, const VMCPtr& vmc_,
                                           PidInterface& length_pid, PidInterface& angle_pid,
                                           PidInterface& angle_vel_pid,
                                           const LegOrientation& leg_orientation, double period,
                                           double feedforward_force)
{
  LegCommand cmd{ 0.0, 0.0, { 0.0, 0.0 } };
  const auto& leg_pos = vmc_->getPos();
  const auto& leg_spd = vmc_->getSpd();

  cmd.force = length_pid.computeCommand(desired_length - leg_pos.L0, period) + feedforward_force;
  cmd.force = std::abs(cmd.force) > 250 ? std::copysign(250.0, cmd.force) : cmd.force;
  if (leg_orientation == LegOrientation::BEHIND || leg_orientation == LegOrientation::UNDER)
  {
    cmd.torque = angle_pid.computeCommand(-shortest_angular_distance(desired_angle, leg_pos.theta), period);
  }
  else
  {
    cmd.torque = angle_vel_pid.computeCommand(-5 - leg_spd.dTheta, period);
  }
  vmc_->leg_conv(cmd.force, cmd.torque, cmd.input);
  return cmd;
}
}  // namespace rm_chassis_controllers
