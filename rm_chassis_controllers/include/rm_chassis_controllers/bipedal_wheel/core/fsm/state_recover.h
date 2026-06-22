#pragma once

#include "bipedal_wheel/core/fsm/fsm_context.h"
#include "bipedal_wheel/core/helper_functions.h"
#include <cmath>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class Recover
{
  enum RecoveryChassisState
  {
    ForwardSlip,
    BackwardSlip
  };

  enum LegRecoveryState
  {
    NotReady,
    Ready
  };

public:
  Recover() = default;

  void execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx)
  {
    if (!ctx.balance_state_changed)
    {
      ctx.logger.info("[balance] Enter RECOVER");
      detectd_flag_ = false;
      ctx.balance_state_changed = true;
    }

    const auto& left_pos = ctx.left_vmc->getPos();
    const auto& right_pos = ctx.right_vmc->getPos();
    const auto& left_spd = ctx.left_vmc->getSpd();
    const auto& right_spd = ctx.right_vmc->getSpd();
    const auto& chassis_state = ctx.sens_in.chassis_state;

    // Pitch and Theta vectors for left and right legs
    double x_left_dtheta = left_spd.dTheta - chassis_state.angular_vel.y();
    double x_right_d_pitch = chassis_state.angular_vel.y();

    if (!detectd_flag_ && std::abs(x_left_dtheta) < 0.2 && std::abs(x_right_d_pitch) < 0.2 &&
        std::abs(chassis_state.angular_vel.y()) < 0.1)
    {
      detectChassisStateToRecover(chassis_state.pitch, ctx.logger);
      detectLegRecoveryState(left_recovery_leg_, left_pos.theta, ctx.logger);
      detectLegRecoveryState(right_recovery_leg_, right_pos.theta, ctx.logger);
      detectd_flag_ = true;
      ctx.recovery_leg_spd_turnback = false;
      leg_recovery_velocity_ =
          (recovery_chassis_state_ == BackwardSlip) ? -leg_recovery_velocity_const_ : leg_recovery_velocity_const_;
    }

    LegCommand left_cmd{};
    LegCommand right_cmd{};

    leg_theta_diff_ = shortest_angular_distance(left_pos.theta, right_pos.theta);
    double T_theta_diff = 0.0, feedforward_force = 0.0;

    if (ctx.cmd_in.base_state != 4 && detectd_flag_)  // base_state 4 corresponds to PROTECT/SITDOWN/etc.
    {
      double left_force =
          ctx.pid_legs[0]->computeCommand(desired_leg_length_ - left_pos.L0, ctx.dt) + feedforward_force;
      double right_force =
          ctx.pid_legs[1]->computeCommand(desired_leg_length_ - right_pos.L0, ctx.dt) + feedforward_force;
      double left_torque = 0.0;
      double right_torque = 0.0;

      if (ctx.recovery_leg_spd_turnback)
      {
        ctx.recovery_leg_spd_turnback = false;
        leg_recovery_velocity_ = -leg_recovery_velocity_;
      }

      double left_input[2] = { 0.0, 0.0 };
      double right_input[2] = { 0.0, 0.0 };

      if (chassis_state.roll < -0.5)
      {
        left_torque = ctx.pid_thetas[2]->computeCommand(leg_recovery_velocity_ - left_spd.dTheta, ctx.dt);
        right_torque = ctx.pid_thetas[3]->computeCommand(0.0 - right_spd.dTheta, ctx.dt);
        left_leg_recovery_feed_forward_ = 2.0 * leg_recovery_velocity_;
        right_leg_recovery_feed_forward_ = 0.0;
        ctx.left_vmc->leg_conv(left_force, left_leg_recovery_feed_forward_ + left_torque, left_input);
        ctx.right_vmc->leg_conv(right_force, right_leg_recovery_feed_forward_ + right_torque, right_input);
      }
      else if (chassis_state.roll > 0.5)
      {
        left_torque = ctx.pid_thetas[2]->computeCommand(0.0 - left_spd.dTheta, ctx.dt);
        right_torque = ctx.pid_thetas[3]->computeCommand(leg_recovery_velocity_ - right_spd.dTheta, ctx.dt);
        left_leg_recovery_feed_forward_ = 0.0;
        right_leg_recovery_feed_forward_ = 2.0 * leg_recovery_velocity_;
        ctx.left_vmc->leg_conv(left_force, left_leg_recovery_feed_forward_ + left_torque, left_input);
        ctx.right_vmc->leg_conv(right_force, right_leg_recovery_feed_forward_ + right_torque, right_input);
      }
      else
      {
        if (left_recovery_leg_ == NotReady && right_recovery_leg_ == Ready)
        {
          detectLegRecoveryState(left_recovery_leg_, left_pos.theta, ctx.logger);
          detectLegRecoveryState(right_recovery_leg_, right_pos.theta, ctx.logger);
          left_torque = ctx.pid_thetas[2]->computeCommand(leg_recovery_velocity_ - left_spd.dTheta, ctx.dt);
          right_torque = ctx.pid_thetas[3]->computeCommand(0.0 - right_spd.dTheta, ctx.dt);
          left_leg_recovery_feed_forward_ = 2.0 * leg_recovery_velocity_;
          right_leg_recovery_feed_forward_ = 0.0;
          ctx.left_vmc->leg_conv(left_force, left_leg_recovery_feed_forward_ + left_torque, left_input);
          ctx.right_vmc->leg_conv(right_force, right_leg_recovery_feed_forward_ + right_torque, right_input);
        }
        if (left_recovery_leg_ == Ready && right_recovery_leg_ == NotReady)
        {
          detectLegRecoveryState(left_recovery_leg_, left_pos.theta, ctx.logger);
          detectLegRecoveryState(right_recovery_leg_, right_pos.theta, ctx.logger);
          left_torque = ctx.pid_thetas[2]->computeCommand(0.0 - left_spd.dTheta, ctx.dt);
          right_torque = ctx.pid_thetas[3]->computeCommand(leg_recovery_velocity_ - right_spd.dTheta, ctx.dt);
          left_leg_recovery_feed_forward_ = 0.0;
          right_leg_recovery_feed_forward_ = 2.0 * leg_recovery_velocity_;
          ctx.left_vmc->leg_conv(left_force, left_leg_recovery_feed_forward_ + left_torque, left_input);
          ctx.right_vmc->leg_conv(right_force, right_leg_recovery_feed_forward_ + right_torque, right_input);
        }
        if (std::abs(leg_theta_diff_) < 0.4)
        {
          if ((left_recovery_leg_ == Ready && right_recovery_leg_ == Ready) ||
              (left_recovery_leg_ == NotReady && right_recovery_leg_ == NotReady))
          {
            T_theta_diff = ctx.pid_theta_diff->computeCommand(leg_theta_diff_, ctx.dt);
            detectLegRecoveryState(left_recovery_leg_, left_pos.theta, ctx.logger);
            detectLegRecoveryState(right_recovery_leg_, right_pos.theta, ctx.logger);
            left_torque = ctx.pid_thetas[2]->computeCommand(leg_recovery_velocity_ - left_spd.dTheta, ctx.dt);
            right_torque = ctx.pid_thetas[3]->computeCommand(leg_recovery_velocity_ - right_spd.dTheta, ctx.dt);
            left_leg_recovery_feed_forward_ = 2.0 * leg_recovery_velocity_;
            right_leg_recovery_feed_forward_ = left_leg_recovery_feed_forward_;
            ctx.left_vmc->leg_conv(left_force, left_leg_recovery_feed_forward_ + left_torque + T_theta_diff, left_input);
            ctx.right_vmc->leg_conv(right_force, right_leg_recovery_feed_forward_ + right_torque - T_theta_diff,
                                    right_input);
          }
        }
      }

      left_cmd.hip.effort = left_input[0];
      left_cmd.knee.effort = left_input[1];
      left_cmd.wheel.effort = 0.0;

      right_cmd.hip.effort = right_input[0];
      right_cmd.knee.effort = right_input[1];
      right_cmd.wheel.effort = 0.0;
    }

    ctx.control_output.leg_cmd[LEFT] = left_cmd;
    ctx.control_output.leg_cmd[RIGHT] = right_cmd;

    // Exit conditions
    if (std::abs(chassis_state.pitch) < 0.2 && chassis_state.linear_acc.z() > 5.0 && !ctx.overturn)
    {
      ctx.current_physical_state = RobotPhysicalState::FALLEN;  // FALLEN maps to SitDown
      ctx.balance_state_changed = false;
      ctx.overturn = false;  // maps to clearRecoveryFlag()
      ctx.logger.info("[balance] Exit RECOVER");
    }
  }

private:
  void detectChassisStateToRecover(double pitch, LoggerType& logger)
  {
    if (pitch > 0.45 && pitch < M_PI)
    {
      logger.info("forward");
      recovery_chassis_state_ = RecoveryChassisState::ForwardSlip;
    }
    else if (pitch < -0.45 && pitch > -M_PI)
    {
      logger.info("back");
      recovery_chassis_state_ = RecoveryChassisState::BackwardSlip;
    }
  }

  void detectLegRecoveryState(LegRecoveryState& leg_recovery_state, double leg_pos, LoggerType& logger)
  {
    LegRecoveryState old_state = leg_recovery_state;
    if (recovery_chassis_state_ == RecoveryChassisState::ForwardSlip)
    {
      if ((leg_pos < M_PI && leg_pos > M_PI - 0.3) || (leg_pos < (-M_PI_2 + 0.4) && leg_pos > -M_PI))
      {
        leg_recovery_state = Ready;
      }
      else
      {
        leg_recovery_state = NotReady;
      }
    }
    else
    {
      if ((leg_pos > 0.5 && leg_pos < M_PI) || (leg_pos < (-M_PI_2 + 1.0) && leg_pos > -M_PI))
      {
        leg_recovery_state = Ready;
      }
      else
      {
        leg_recovery_state = NotReady;
      }
    }
    if (leg_recovery_state != old_state)
    {
      logger.info("Leg recovery state: " + std::to_string(leg_recovery_state));
    }
  }

  double leg_recovery_velocity_ = 5.0;
  double threshold_ = 0.05;
  double leg_theta_diff_ = 0.0;
  double desired_leg_length_ = 0.36;
  double left_leg_recovery_feed_forward_ = 10.0;
  double right_leg_recovery_feed_forward_ = 10.0;
  static constexpr double leg_recovery_velocity_const_ = 5.0;
  LegRecoveryState left_recovery_leg_ = NotReady;
  LegRecoveryState right_recovery_leg_ = NotReady;
  RecoveryChassisState recovery_chassis_state_ = ForwardSlip;
  bool detectd_flag_ = false;
};

}  // namespace bipedal_wheel_core
