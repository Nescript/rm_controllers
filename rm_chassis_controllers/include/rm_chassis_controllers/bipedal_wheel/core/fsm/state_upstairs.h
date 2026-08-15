#pragma once

#include "bipedal_wheel/core/fsm/fsm_context.h"
#include "bipedal_wheel/core/helper_functions.h"
#include <cmath>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class Upstairs
{
public:
  Upstairs() = default;

  void execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx)
  {
    const auto& left_pos = ctx.left_vmc->getPos();
    const auto& right_pos = ctx.right_vmc->getPos();

    if (!ctx.balance_state_changed)
    {
      ctx.logger.info("[balance] Enter Upstairs");
      ctx.balance_state_changed = true;
      ctx.complete_stand = false;
      detectLegState(left_pos.theta, ctx.config.threshold, left_leg_orientation, ctx.logger);
      detectLegState(right_pos.theta, ctx.config.threshold, right_leg_orientation, ctx.logger);
    }

    double left_spring_force =
        -f_spring_force(left_pos.L0, ctx.left_vmc->getL1(), ctx.left_vmc->getL2(), ctx.config.spring);
    double right_spring_force =
        -f_spring_force(right_pos.L0, ctx.right_vmc->getL1(), ctx.right_vmc->getL2(), ctx.config.spring);

    double length_des_l = ctx.config.threshold.upstair_des_length;
    double length_des_r = ctx.config.threshold.upstair_des_length;
    double theta_des_l = ctx.config.threshold.upstair_des_theta;
    double theta_des_r = ctx.config.threshold.upstair_des_theta;

    LegCommand left_cmd{};
    LegCommand right_cmd{};

    left_cmd = computePidLegCommand(length_des_l, theta_des_l, ctx.left_vmc, *ctx.pid_legs[0], *ctx.pid_thetas[0],
                                    *ctx.pid_thetas[2], left_leg_orientation, ctx.dt, left_spring_force);
    right_cmd = computePidLegCommand(length_des_r, theta_des_r, ctx.right_vmc, *ctx.pid_legs[1], *ctx.pid_thetas[1],
                                     *ctx.pid_thetas[3], right_leg_orientation, ctx.dt, right_spring_force);

    ctx.control_output.leg_cmd[LEFT] = left_cmd;
    ctx.control_output.leg_cmd[RIGHT] = right_cmd;

    // Exit conditions
    if (left_pos.theta > ctx.config.threshold.upstair_exit_theta_threshold &&
        right_pos.theta > ctx.config.threshold.upstair_exit_theta_threshold &&
        left_pos.L0 < ctx.config.threshold.upstair_exit_length_threshold &&
        right_pos.L0 < ctx.config.threshold.upstair_exit_length_threshold)
    {
      ctx.logger.publishUpstairStatus(true);
      ctx.current_physical_state = RobotPhysicalState::GETTING_UP;  // GETTING_UP maps to StandUp
      ctx.balance_state_changed = false;
      ctx.logger.info("[balance] Exit Upstairs");
    }
  }

private:
  void detectLegState(double theta, const LegStateThresholdParams& threshold, LegOrientation& leg_state,
                      LoggerType& logger)
  {
    if (theta > threshold.under_lower && theta < threshold.under_upper)
      leg_state = LegOrientation::UNDER;
    else if ((theta < threshold.front_lower && theta > -M_PI) || (theta < M_PI && theta > threshold.front_upper))
      leg_state = LegOrientation::FRONT;
    else if (theta > threshold.behind_lower && theta < threshold.behind_upper)
      leg_state = LegOrientation::BEHIND;

    switch (leg_state)
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

  LegCommand computePidLegCommand(double desired_length, double desired_angle, VMC* vmc_, PidType& length_pid,
                                  PidType& angle_pid, PidType& angle_vel_pid, const LegOrientation& leg_orientation,
                                  double dt, double feedforward_force)
  {
    LegCommand cmd{};
    const auto& leg_pos = vmc_->getPos();
    const auto& leg_spd = vmc_->getSpd();

    double force = length_pid.computeCommand(desired_length - leg_pos.L0, dt) + feedforward_force;
    force = std::abs(force) > 250 ? std::copysign(1.0, force) * 250 : force;
    double torque = 0.0;
    if (leg_orientation == LegOrientation::BEHIND || leg_orientation == LegOrientation::UNDER)
    {
      torque = angle_pid.computeCommand(-shortest_angular_distance(desired_angle, leg_pos.theta), dt);
    }
    else
    {
      torque = angle_vel_pid.computeCommand(-5.0 - leg_spd.dTheta, dt);
    }
    double input[2] = { 0.0, 0.0 };
    vmc_->leg_conv(force, torque, input);
    cmd.hip.effort = input[0];
    cmd.knee.effort = input[1];
    cmd.wheel.effort = 0.0;
    return cmd;
  }

  LegOrientation left_leg_orientation = LegOrientation::UNDER;
  LegOrientation right_leg_orientation = LegOrientation::UNDER;
};

}  // namespace bipedal_wheel_core
