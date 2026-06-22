#pragma once

#include "bipedal_wheel/core/fsm/fsm_context.h"
#include "bipedal_wheel/core/helper_functions.h"
#include <cmath>
#include <memory>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class Protect
{
public:
  Protect()
  {
    double leg_len_acc = 20.0, leg_theta_acc = 10.0;
    ramp_length_des_l_ = std::make_shared<RampFilterType>(leg_len_acc, 0.001);
    ramp_length_des_r_ = std::make_shared<RampFilterType>(leg_len_acc, 0.001);
    ramp_angle_des_l_ = std::make_shared<RampFilterType>(leg_theta_acc, 0.001);
    ramp_angle_des_r_ = std::make_shared<RampFilterType>(leg_theta_acc, 0.001);
  }

  void execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx)
  {
    if (!ctx.balance_state_changed)
    {
      ctx.logger.info("[balance] Enter PROTECT");
      ctx.balance_state_changed = true;
    }

    const auto& left_pos = ctx.left_vmc->getPos();
    const auto& right_pos = ctx.right_vmc->getPos();
    const auto& chassis_state = ctx.sens_in.chassis_state;

    double left_wheel_desired_vel = 0.0, right_wheel_desired_vel = 0.0;
    left_wheel_desired_vel = ctx.cmd_in.vel_cmd.x() - ctx.cmd_in.vel_cmd.z() * ctx.config.chassis_geometry.wheel_track;
    right_wheel_desired_vel = ctx.cmd_in.vel_cmd.x() + ctx.cmd_in.vel_cmd.z() * ctx.config.chassis_geometry.wheel_track;

    length_des_l = length_des_r = 0.11;
    theta_des_l = theta_des_r = 0.0;

    ramp_length_des_l_->input(length_des_l);
    ramp_angle_des_l_->input(theta_des_l);
    ramp_length_des_r_->input(length_des_r);
    ramp_angle_des_r_->input(theta_des_r);

    length_des_l = ramp_length_des_l_->output();
    theta_des_l = ramp_angle_des_l_->output();
    length_des_r = ramp_length_des_r_->output();
    theta_des_r = ramp_angle_des_r_->output();

    LegCommand left_cmd{};
    LegCommand right_cmd{};
    double F_pid_left = 0.0, F_pid_right = 0.0;

    F_pid_left = ctx.pid_legs[LEFT]->computeCommand(length_des_l - left_pos.L0, ctx.dt);
    F_pid_right = ctx.pid_legs[RIGHT]->computeCommand(length_des_r - right_pos.L0, ctx.dt);
    F_pid_left = std::abs(F_pid_left) > 200 ? std::copysign(1.0, F_pid_left) * 200 : F_pid_left;
    F_pid_right = std::abs(F_pid_right) > 200 ? std::copysign(1.0, F_pid_right) * 200 : F_pid_right;

    double left_force =
        F_pid_left - f_spring_force(left_pos.L0, ctx.left_vmc->getL1(), ctx.left_vmc->getL2(), ctx.config.spring);
    double right_force =
        F_pid_right - f_spring_force(right_pos.L0, ctx.right_vmc->getL1(), ctx.right_vmc->getL2(), ctx.config.spring);

    double T_theta_diff = ctx.pid_theta_diff->computeCommand(right_pos.theta - left_pos.theta, ctx.dt);
    double left_torque = ctx.pid_thetas[0]->computeCommand(theta_des_l - left_pos.theta, ctx.dt) + T_theta_diff;
    double right_torque = ctx.pid_thetas[1]->computeCommand(theta_des_r - right_pos.theta, ctx.dt) - T_theta_diff;

    double T_yaw = ctx.pid_yaw_vel->computeCommand(ctx.cmd_in.vel_cmd.z() - chassis_state.angular_vel.z(), ctx.dt);
    double left_wheel_cmd =
        ctx.pid_wheels[0]->computeCommand(left_wheel_desired_vel - ctx.sens_in.leg_state[LEFT].wheel.vel, ctx.dt) -
        T_yaw;
    double right_wheel_cmd =
        ctx.pid_wheels[1]->computeCommand(right_wheel_desired_vel - ctx.sens_in.leg_state[RIGHT].wheel.vel, ctx.dt) +
        T_yaw;

    double left_input[2] = { 0.0, 0.0 };
    double right_input[2] = { 0.0, 0.0 };
    ctx.left_vmc->leg_conv(left_force, left_torque, left_input);
    ctx.right_vmc->leg_conv(right_force, right_torque, right_input);

    left_cmd.hip.effort = left_input[0];
    left_cmd.knee.effort = left_input[1];
    left_cmd.wheel.effort = left_wheel_cmd;

    right_cmd.hip.effort = right_input[0];
    right_cmd.knee.effort = right_input[1];
    right_cmd.wheel.effort = right_wheel_cmd;

    ctx.control_output.leg_cmd[LEFT] = left_cmd;
    ctx.control_output.leg_cmd[RIGHT] = right_cmd;

    // Exit conditions
    if (std::abs(chassis_state.pitch) < 0.3 && std::abs(chassis_state.angular_vel.y()) < 0.2 &&
        std::abs(left_pos.theta + right_pos.theta) / 2.0 < 0.2)
    {
      ctx.current_physical_state = RobotPhysicalState::STAND;  // STAND maps to Normal
      ctx.balance_state_changed = false;
      ctx.logger.info("[balance] Exit PROTECT");
    }
    else if (std::abs(chassis_state.angular_vel.y()) < 0.1 && ctx.overturn && ctx.cmd_in.base_state != 3)  // FALLEN = 3
    {
      ctx.current_physical_state = RobotPhysicalState::GETTING_UP;  // GETTING_UP maps to Recover
      ctx.balance_state_changed = false;
      ctx.logger.info("[balance] Exit PROTECT");
    }
  }

private:
  double theta_des_l = 0.0, theta_des_r = 0.0, length_des_l = 0.11, length_des_r = 0.11;
  std::shared_ptr<RampFilterType> ramp_length_des_l_;
  std::shared_ptr<RampFilterType> ramp_length_des_r_;
  std::shared_ptr<RampFilterType> ramp_angle_des_l_;
  std::shared_ptr<RampFilterType> ramp_angle_des_r_;
};

}  // namespace bipedal_wheel_core
