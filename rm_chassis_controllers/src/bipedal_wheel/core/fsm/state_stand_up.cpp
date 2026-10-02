#include "bipedal_wheel/core/fsm/state_stand_up.h"
#include "bipedal_wheel/core/helper_functions.h"
#include "bipedal_wheel/controllers.h"
#include <cmath>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
StandUp<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::StandUp()
{
  ramp_length_des_l_ = std::make_shared<RampFilterType>(50.0, 0.001);
  ramp_length_des_r_ = std::make_shared<RampFilterType>(50.0, 0.001);
  ramp_angle_des_l_ = std::make_shared<RampFilterType>(5.0, 0.001);
  ramp_angle_des_r_ = std::make_shared<RampFilterType>(5.0, 0.001);
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
void StandUp<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx)
{
  const auto& left_pos = ctx.left_vmc->getPos();
  const auto& right_pos = ctx.right_vmc->getPos();
  const auto& left_spd = ctx.left_vmc->getSpd();
  const auto& right_spd = ctx.right_vmc->getSpd();

  double pitch = ctx.sens_in.chassis_state.pitch;
  double d_pitch = ctx.sens_in.chassis_state.angular_vel.y();

  double left_theta_total = left_pos.theta + pitch;
  double left_dtheta_total = left_spd.dTheta + d_pitch;
  double right_theta_total = right_pos.theta + pitch;
  double right_dtheta_total = right_spd.dTheta + d_pitch;

  if (!ctx.balance_state_changed)
  {
    ctx.logger.info("[balance] Enter STAND_UP");
    ctx.balance_state_changed = true;
    ctx.complete_stand = false;
    left_arrive_flag_ = right_arrive_flag_ = false;

    detectLegState(left_theta_total, ctx.config.threshold, left_leg_orientation, ctx.logger);
    detectLegState(right_theta_total, ctx.config.threshold, right_leg_orientation, ctx.logger);
  }

  double left_spring_force =
      -(((2094.45f * left_pos.L0 - 3091.28f) * left_pos.L0 + 1408.375f) * left_pos.L0 - 80.91f);
  double right_spring_force =
      -(((2094.45f * right_pos.L0 - 3091.28f) * right_pos.L0 + 1408.375f) * right_pos.L0 - 80.91f);

  LegCommand left_cmd{};
  LegCommand right_cmd{};

  // setUpLegMotion takes inputs and outputs
  setUpLegMotion(left_theta_total, left_dtheta_total, left_pos.L0, left_pos.theta, ctx.config.threshold,
                 right_leg_orientation, left_leg_orientation, left_leg_command_, left_stop_,
                 left_arrive_flag_, left_arrive_time_);
  setUpLegMotion(right_theta_total, right_dtheta_total, right_pos.L0, right_pos.theta, ctx.config.threshold,
                 left_leg_orientation, right_leg_orientation, right_leg_command_, right_stop_,
                 right_arrive_flag_, right_arrive_time_);

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
    left_cmd = computePidLegCommand(left_leg_command_, ctx.left_vmc, *ctx.pid_legs[0], *ctx.pid_thetas[0],
                                    *ctx.pid_thetas[2], left_leg_orientation, ctx.dt, left_spring_force);
  }
  if (!right_stop_)
  {
    right_cmd = computePidLegCommand(right_leg_command_, ctx.right_vmc, *ctx.pid_legs[1], *ctx.pid_thetas[1],
                                     *ctx.pid_thetas[3], right_leg_orientation, ctx.dt, right_spring_force);
  }

  ctx.control_output.leg_cmd[LEFT] = left_cmd;
  ctx.control_output.leg_cmd[RIGHT] = right_cmd;

  // Exit conditions
  if ((((std::abs(left_pos.theta) < 0.3 && left_leg_orientation == LegOrientation::BEHIND)) &&
       ((std::abs(right_pos.theta) < 0.3 && right_leg_orientation == LegOrientation::BEHIND))) ||
      ((std::abs(left_pos.theta) < 0.3 && left_leg_orientation == LegOrientation::UNDER) &&
       (std::abs(right_pos.theta) < 0.3 && right_leg_orientation == LegOrientation::UNDER)))
  {
    ctx.current_physical_state = RobotPhysicalState::STAND;
    ctx.balance_state_changed = false;
    ctx.logger.info("[balance] Exit STAND_UP");
  }

  if (ctx.overturn)
  {
    ctx.current_physical_state = RobotPhysicalState::GETTING_UP;
    ctx.balance_state_changed = false;
    ctx.logger.info("[balance] Exit STAND_UP (overturn)");
  }
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
void StandUp<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::setUpLegMotion(
    double theta_total, double dtheta_total, double leg_length, double leg_theta,
    const LegStateThresholdParams& threshold, const LegOrientation& other_leg_orientation,
    LegOrientation& leg_orientation, StandUpLegCommand& legCommand, bool& stop_flag, bool& arrive_flag,
    double arrive_time)
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
      legCommand.desired_angle = M_PI_2 - 0.35;
      legCommand.desired_length = 0.34;
      legCommand.desired_angle_vel = 0.0;
      if (leg_length > 0.30)
        legCommand.desired_angle_vel = -5.0;
      if (std::abs(legCommand.desired_angle - leg_theta) < 0.5)
      {
        legCommand.desired_angle_vel = -1.5;
      }
      // Preserve the verified rm_ws motion and its pass-by-value arrival timestamp.
      // Do not turn this into a continuous stationary dwell: that changes stand-up.
      if (std::abs(dtheta_total) < 0.1)
      {
        if (theta_total > 0.0 && theta_total < M_PI_2 + 0.4f)
        {
          legCommand.desired_angle_vel = -0.5f;
          if (!arrive_flag)
          {
            arrive_flag = true;
            arrive_time = ros::Time::now().toSec();
          }
          if (ros::Time::now().toSec() - arrive_time > threshold.arrive_time_threshold)
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
        legCommand.desired_length = 0.12;
        legCommand.desired_angle = 0.0;
      }
      break;
  }
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
void StandUp<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::detectLegState(double theta, const LegStateThresholdParams& threshold, LegOrientation& leg_orientation,
                      LoggerType& logger)
{
  if (theta > threshold.under_lower && theta < threshold.under_upper)
    leg_orientation = LegOrientation::UNDER;
  else if ((theta < threshold.front_lower && theta > -M_PI) || (theta < M_PI && theta > threshold.front_upper))
    leg_orientation = LegOrientation::FRONT;
  else if (theta > threshold.behind_lower && theta < threshold.behind_upper)
    leg_orientation = LegOrientation::BEHIND;

  switch (leg_orientation)
  {
    case LegOrientation::UNDER:
      logger.info("[balance] theta: " + std::to_string(theta) + " Leg state: UNDER");
      break;
    case LegOrientation::FRONT:
      logger.info("[balance] theta: " + std::to_string(theta) + " Leg state: FRONT");
      break;
    case LegOrientation::BEHIND:
      logger.info("[balance] theta: " + std::to_string(theta) + " Leg state: BEHIND");
      break;
  }
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
LegCommand StandUp<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::computePidLegCommand(const StandUpLegCommand& leg_command, VMC* vmc_, PidType& length_pid,
                                  PidType& angle_pid, PidType& angle_vel_pid, const LegOrientation& leg_orientation,
                                  double dt, double feedforward_force)
{
  LegCommand cmd{};

  const auto& leg_pos = vmc_->getPos();
  const auto& leg_spd = vmc_->getSpd();

  double Tp_leg_comp = 0.0, F_leg_comp = 0.0, beta = 0.0;
  double leg_mass = 2.0;
  double G_leg = leg_mass * 9.81;
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

  double F_pid_force = length_pid.computeCommand(leg_command.desired_length - leg_pos.L0, dt);
  F_pid_force = std::abs(F_pid_force) > 100 ? std::copysign(1.0, F_pid_force) * 100 : F_pid_force;
  double force = F_pid_force + feedforward_force;
  double torque = 0.0;
  if (leg_orientation == LegOrientation::BEHIND || leg_orientation == LegOrientation::UNDER)
  {
    torque = angle_pid.computeCommand(-shortest_angular_distance(leg_command.desired_angle, leg_pos.theta), dt);
  }
  else
  {
    torque = angle_vel_pid.computeCommand(leg_command.desired_angle_vel - leg_spd.dTheta, dt);
  }
  double input[2] = { 0.0, 0.0 };
  vmc_->leg_conv(force + F_leg_comp, torque + Tp_leg_comp, input);
  cmd.hip.effort = input[0];
  cmd.knee.effort = input[1];
  cmd.wheel.effort = 0.0;
  return cmd;
}

// Explicit instantiation for the ROS controller's injected types.
template class StandUp<rm_chassis_controllers::RosPidWrapper, rm_chassis_controllers::RosLoggerWrapper,
                      RampFilter<double>, MovingAverageFilter<double>>;

}  // namespace bipedal_wheel_core
