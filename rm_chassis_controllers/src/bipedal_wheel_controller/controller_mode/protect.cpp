//
// Created by wk on 2026/5/13.
//

#include "bipedal_wheel_controller/controller_mode/protect.h"
#include "bipedal_wheel_controller/math_helpers.h"

namespace rm_chassis_controllers
{
Protect::Protect(PidInterface* left_leg, PidInterface* right_leg, PidInterface* left_theta, PidInterface* right_theta,
                 PidInterface* left_wheel, PidInterface* right_wheel, PidInterface* theta_diff, PidInterface* yaw_vel)
  : pid_left_leg_(left_leg)
  , pid_right_leg_(right_leg)
  , pid_left_theta_(left_theta)
  , pid_right_theta_(right_theta)
  , pid_left_wheel_(left_wheel)
  , pid_right_wheel_(right_wheel)
  , pid_theta_diff_(theta_diff)
  , pid_yaw_vel_(yaw_vel)
{
  double leg_len_acc = 20, leg_theta_acc = 10;
  ramp_length_des_l_ = std::make_shared<RampFilter<double>>(leg_len_acc, 0.001);
  ramp_length_des_r_ = std::make_shared<RampFilter<double>>(leg_len_acc, 0.001);
  ramp_angle_des_l_ = std::make_shared<RampFilter<double>>(leg_theta_acc, 0.001);
  ramp_angle_des_r_ = std::make_shared<RampFilter<double>>(leg_theta_acc, 0.001);
}

LogicOutput Protect::execute(const LogicInput& input)
{
  LogicOutput output;
  if (!input.state_changed)
  {
    output.state_changed_requested = true;
  }
  const auto& left_leg_state = input.leg_state[LEFT];
  const auto& right_leg_state = input.leg_state[RIGHT];
  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();
  const auto& chassis_geometry_params = input.chassis_geometry_params;
  const auto& chassis_state = input.chassis_state;
  const auto& vel_cmd_ = input.instruction;
  const auto& spring_params_ = input.spring_params;

  double left_wheel_desired_vel{}, right_wheel_desired_vel{};
  left_wheel_desired_vel = vel_cmd_.x_vel_cmd - vel_cmd_.yaw_vel_cmd * chassis_geometry_params->wheel_track;
  right_wheel_desired_vel = vel_cmd_.x_vel_cmd + vel_cmd_.yaw_vel_cmd * chassis_geometry_params->wheel_track;

  double length_des_l = 0.11f, length_des_r = 0.11f;
  double theta_des_l = 0.0f, theta_des_r = 0.0f;
  ramp_length_des_l_->input(length_des_l);
  ramp_angle_des_l_->input(theta_des_l);
  ramp_length_des_r_->input(length_des_r);
  ramp_angle_des_r_->input(theta_des_r);
  length_des_l = ramp_length_des_l_->output();
  theta_des_l = ramp_angle_des_l_->output();
  length_des_r = ramp_length_des_r_->output();
  theta_des_r = ramp_angle_des_r_->output();

  LegCommand left_cmd{}, right_cmd{};
  double F_pid_left{}, F_pid_right{};
  F_pid_left = pid_left_leg_->computeCommand(length_des_l - left_pos.L0, input.period);
  F_pid_right = pid_right_leg_->computeCommand(length_des_r - right_pos.L0, input.period);
  F_pid_left = std::abs(F_pid_left) > 200 ? std::copysign(200.0, F_pid_left) : F_pid_left;
  F_pid_right = std::abs(F_pid_right) > 200 ? std::copysign(200.0, F_pid_right) : F_pid_right;
  
  double left_spring_force = f_spring_force(left_pos.L0, left_leg_state.vmc->getL1(), left_leg_state.vmc->getL2(),
                                           spring_params_->f_spring, spring_params_->s2, spring_params_->s3, spring_params_->alpha_s);
  double right_spring_force = f_spring_force(right_pos.L0, right_leg_state.vmc->getL1(), right_leg_state.vmc->getL2(),
                                            spring_params_->f_spring, spring_params_->s2, spring_params_->s3, spring_params_->alpha_s);

  left_cmd.force = F_pid_left - left_spring_force;
  right_cmd.force = F_pid_right - right_spring_force;
  double T_theta_diff = pid_theta_diff_->computeCommand(right_pos.theta - left_pos.theta, input.period);
  left_cmd.torque = pid_left_theta_->computeCommand(theta_des_l - left_pos.theta, input.period) + T_theta_diff;
  right_cmd.torque = pid_right_theta_->computeCommand(theta_des_r - right_pos.theta, input.period) - T_theta_diff;
  double T_yaw = pid_yaw_vel_->computeCommand(vel_cmd_.yaw_vel_cmd - chassis_state.angular_vel.z(), input.period);
  
  // Need wheel velocities in input
  double left_wheel_cmd =
      pid_left_wheel_->computeCommand(left_wheel_desired_vel - input.left_wheel_vel, input.period) - T_yaw;
  double right_wheel_cmd =
      pid_right_wheel_->computeCommand(right_wheel_desired_vel - input.right_wheel_vel, input.period) + T_yaw;

  left_leg_state.vmc->leg_conv(left_cmd.force, left_cmd.torque, left_cmd.input);
  right_leg_state.vmc->leg_conv(right_cmd.force, right_cmd.torque, right_cmd.input);

  output.leg_cmd[LEFT] = left_cmd;
  output.leg_cmd[RIGHT] = right_cmd;
  output.wheel_cmd[LEFT] = left_wheel_cmd;
  output.wheel_cmd[RIGHT] = right_wheel_cmd;

  // Exit
  if (std::abs(chassis_state.pitch) < 0.3f && std::abs(chassis_state.angular_vel.y()) < 0.2f &&
      std::abs(left_pos.theta + right_pos.theta) / 2.0f < 0.2f)
  {
    output.next_mode = BalanceMode::NORMAL;
    output.state_changed_requested = false;
  }
  else if (std::abs(chassis_state.angular_vel.y()) < 0.1 && input.overturn &&
           input.base_state != 4) // Assume 4 is FALLEN
  {
    output.state_changed_requested = false;
    output.next_mode = BalanceMode::RECOVER;
  }
  
  return output;
}

}  // namespace rm_chassis_controllers
