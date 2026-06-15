//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/stand_up.h"
#include "bipedal_wheel_controller/math_helpers.h"

namespace rm_chassis_controllers
{
StandUp::StandUp(PidInterface* pid_left_leg, PidInterface* pid_right_leg, PidInterface* pid_left_theta,
                 PidInterface* pid_right_theta, PidInterface* pid_left_theta_vel, PidInterface* pid_right_theta_vel)
  : pid_left_leg_(pid_left_leg)
  , pid_right_leg_(pid_right_leg)
  , pid_left_theta_(pid_left_theta)
  , pid_right_theta_(pid_right_theta)
  , pid_left_theta_vel_(pid_left_theta_vel)
  , pid_right_theta_vel_(pid_right_theta_vel)
{
  double leg_len_acc = 50, leg_theta_acc = 7.5;
  ramp_length_des_l_ = std::make_shared<RampFilter<double>>(leg_len_acc, 0.001);
  ramp_length_des_r_ = std::make_shared<RampFilter<double>>(leg_len_acc, 0.001);
  ramp_angle_des_l_ = std::make_shared<RampFilter<double>>(leg_theta_acc, 0.001);
  ramp_angle_des_r_ = std::make_shared<RampFilter<double>>(leg_theta_acc, 0.001);
}

LogicOutput StandUp::execute(const LogicInput& input)
{
  LogicOutput output;
  const auto& left_leg_state = input.leg_state[LEFT];
  const auto& right_leg_state = input.leg_state[RIGHT];
  if (!input.state_changed)
  {
    output.state_changed_requested = true;
    output.complete_stand_requested = false;
    left_arrive_flag_ = right_arrive_flag_ = false;
    StandUp::detectLegState(left_leg_state.x, left_leg_orientation, input.leg_threshold_params);
    StandUp::detectLegState(right_leg_state.x, right_leg_orientation, input.leg_threshold_params);
  }

  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();

  auto spring_params_ = input.spring_params;
  double left_spring_force = -f_spring_force(left_pos.L0, left_leg_state.vmc->getL1(), left_leg_state.vmc->getL2(),
                                            spring_params_->f_spring, spring_params_->s2, spring_params_->s3, spring_params_->alpha_s);
  double right_spring_force = -f_spring_force(right_pos.L0, right_leg_state.vmc->getL1(), right_leg_state.vmc->getL2(),
                                             spring_params_->f_spring, spring_params_->s2, spring_params_->s3, spring_params_->alpha_s);

  setUpLegMotion(left_leg_state.x, right_leg_orientation, left_pos.L0, left_pos.theta, left_leg_orientation,
                 left_leg_command_, left_stop_, left_arrive_flag_, left_arrive_time_, input.time, input.leg_threshold_params);
  setUpLegMotion(right_leg_state.x, left_leg_orientation, right_pos.L0, right_pos.theta, right_leg_orientation,
                 right_leg_command_, right_stop_, right_arrive_flag_, right_arrive_time_, input.time, input.leg_threshold_params);

  ramp_length_des_l_->input(left_leg_command_.desired_length);
  ramp_angle_des_l_->input(left_leg_command_.desired_angle);
  ramp_length_des_r_->input(right_leg_command_.desired_length);
  ramp_angle_des_r_->input(right_leg_command_.desired_angle);
  left_leg_command_.desired_length = ramp_length_des_l_->output();
  left_leg_command_.desired_angle = ramp_angle_des_l_->output();
  right_leg_command_.desired_length = ramp_length_des_r_->output();
  right_leg_command_.desired_angle = ramp_angle_des_r_->output();

  if (!left_stop_)
  {
    output.leg_cmd[LEFT] = computePidLegCommand(left_leg_command_, left_leg_state.vmc, *pid_left_leg_, *pid_left_theta_,
                                              *pid_left_theta_vel_, left_leg_orientation, input.period, left_spring_force);
  }
  else
  {
    output.leg_cmd[LEFT] = { 0, 0, { 0., 0. } };
  }
  if (!right_stop_)
  {
    output.leg_cmd[RIGHT] = computePidLegCommand(right_leg_command_, right_leg_state.vmc, *pid_right_leg_, *pid_right_theta_,
                                               *pid_right_theta_vel_, right_leg_orientation, input.period, right_spring_force);
  }
  else
  {
    output.leg_cmd[RIGHT] = { 0, 0, { 0., 0. } };
  }
  
  output.wheel_cmd[LEFT] = output.wheel_cmd[RIGHT] = 0.0;

  // Exit
  if ((((std::abs(left_pos.theta) < 0.3f && left_leg_orientation == LegOrientation::BEHIND)) &&
       ((std::abs(right_pos.theta) < 0.3f && right_leg_orientation == LegOrientation::BEHIND))) ||
      ((std::abs(left_pos.theta) < 0.3f && left_leg_orientation == LegOrientation::UNDER) &&
       (std::abs(right_pos.theta) < 0.3f && right_leg_orientation == LegOrientation::UNDER)))
  {
    output.next_mode = BalanceMode::NORMAL;
    output.state_changed_requested = false;
  }
  if (input.overturn)
  {
    output.next_mode = BalanceMode::RECOVER;
    output.state_changed_requested = false;
  }
  
  return output;
}

void StandUp::setUpLegMotion(const Eigen::Matrix<double, STATE_DIM, 1>& x, const LegOrientation& other_leg_orientation,
                             const double& leg_length, const double& leg_theta, LegOrientation& leg_orientation,
                             StandUpLegCommand& legCommand, bool& stop_flag, bool& arrive_flag, double& arrive_time,
                             double time, const std::shared_ptr<LegStateThresholdParams>& threshold)
{
  switch (leg_orientation)
  {
    case LegOrientation::UNDER:
      stop_flag = false;
      legCommand.desired_angle = leg_theta;
      legCommand.desired_length = 0.34;
      if (leg_length > 0.33)
      {
        leg_orientation = LegOrientation::FRONT;
      }
      break;
    case LegOrientation::FRONT:
      stop_flag = false;
      if (!arrive_flag)
        arrive_flag = false;
      legCommand.desired_angle = M_PI_2 - 0.35;
      legCommand.desired_length = 0.34;
      legCommand.desired_angle_vel = 0.0;
      if (leg_length > 0.30)
        legCommand.desired_angle_vel = -5.0;
      if (std::abs(legCommand.desired_angle - leg_theta) < 0.5)
      {
        legCommand.desired_angle_vel = -1.5f;
      }
      if (std::abs(x[1]) < 0.1)
      {
        if (x[0] > 0 && x[0] < M_PI_2 + 0.4f)
        {
          legCommand.desired_angle_vel = -0.5f;
          if (!arrive_flag)
          {
            arrive_flag = true;
            arrive_time = time;
          }
          if (time - arrive_time > threshold->arrive_time_threshold)
            leg_orientation = LegOrientation::BEHIND;
        }
      }
      break;
    case LegOrientation::BEHIND:
      stop_flag = true;
      legCommand.desired_angle = leg_theta;
      legCommand.desired_length = leg_length;
      if (other_leg_orientation == LegOrientation::BEHIND)
      {
        stop_flag = false;

        legCommand.desired_length = 0.12f;
        legCommand.desired_angle = 0.0f;
      }
      break;
  }
}

void StandUp::detectLegState(const Eigen::Matrix<double, STATE_DIM, 1>& x, LegOrientation& leg_orientation,
                             const std::shared_ptr<LegStateThresholdParams>& threshold)
{
  if (!threshold)
  {
    return;
  }
  if (x[0] > threshold->under_lower && x[0] < threshold->under_upper)
    leg_orientation = LegOrientation::UNDER;
  else if ((x[0] < threshold->front_lower && x[0] > -M_PI) ||
           (x[0] < M_PI && x[0] > threshold->front_upper))
    leg_orientation = LegOrientation::FRONT;
  else if (x[0] > threshold->behind_lower && x[0] < threshold->behind_upper)
    leg_orientation = LegOrientation::BEHIND;
}

LegCommand StandUp::computePidLegCommand(const StandUpLegCommand& leg_command, const VMCPtr& vmc_,
                                         PidInterface& length_pid, PidInterface& angle_pid,
                                         PidInterface& angle_vel_pid,
                                         const LegOrientation& leg_orientation, double period,
                                         double feedforward_force)
{
  LegCommand cmd{ 0.0, 0.0, { 0.0, 0.0 } };

  const auto& leg_pos = vmc_->getPos();
  const auto& leg_spd = vmc_->getSpd();

  double Tp_leg_comp{}, F_leg_comp{}, beta{};
  double leg_mass = 2.0f;
  double G_leg = leg_mass * 9.81f;
  double l_leg = get_LM(leg_pos.L0);
  double theta_leg_offset = get_theta_leg_offset(leg_pos.L0);
  if (leg_pos.theta > -M_PI_2 && leg_pos.theta < M_PI_2)
  {
    beta = leg_pos.theta + theta_leg_offset;
    F_leg_comp = -G_leg * l_leg * std::cos(beta);
  }
  else
  {
    if (leg_pos.theta > -M_PI && leg_pos.theta < -M_PI_2)
    {
      beta = -leg_pos.theta - M_PI - theta_leg_offset;
    }
    else if (leg_pos.theta > M_PI_2 && leg_pos.theta < M_PI)
    {
      beta = M_PI - leg_pos.theta - theta_leg_offset;
    }
    F_leg_comp = G_leg * l_leg * std::cos(beta);
  }
  Tp_leg_comp = G_leg * l_leg * std::sin(beta);

  double F_pid_force = length_pid.computeCommand(leg_command.desired_length - leg_pos.L0, period);
  F_pid_force = std::abs(F_pid_force) > 200 ? std::copysign(200.0, F_pid_force) : F_pid_force;
  cmd.force = F_pid_force + feedforward_force;
  if (leg_orientation == LegOrientation::BEHIND || leg_orientation == LegOrientation::UNDER)
  {
    cmd.torque =
        angle_pid.computeCommand(-shortest_angular_distance(leg_command.desired_angle, leg_pos.theta), period);
  }
  else
  {
    cmd.torque = angle_vel_pid.computeCommand(leg_command.desired_angle_vel - leg_spd.dTheta, period);
  }
  vmc_->leg_conv(cmd.force + F_leg_comp, cmd.torque + Tp_leg_comp, cmd.input);
  return cmd;
}
}  // namespace rm_chassis_controllers
