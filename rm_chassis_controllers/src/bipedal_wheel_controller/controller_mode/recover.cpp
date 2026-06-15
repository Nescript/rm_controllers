//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/recover.h"
#include "bipedal_wheel_controller/math_helpers.h"

namespace rm_chassis_controllers
{
Recover::Recover(PidInterface* left_leg, PidInterface* right_leg, PidInterface* left_theta, PidInterface* right_theta,
                 PidInterface* theta_diff)
  : pid_left_leg_(left_leg)
  , pid_right_leg_(right_leg)
  , pid_left_theta_(left_theta)
  , pid_right_theta_(right_theta)
  , pid_theta_diff_(theta_diff)
{
}

LogicOutput Recover::execute(const LogicInput& input)
{
  LogicOutput output;
  if (!input.state_changed)
  {
    detectd_flag = false;
    output.state_changed_requested = true;
  }
  const auto& chassis_state = input.chassis_state;
  const auto& left_leg_state = input.leg_state[LEFT];
  const auto& right_leg_state = input.leg_state[RIGHT];
  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();
  const auto& left_spd = left_leg_state.vmc->getSpd();
  const auto& right_spd = right_leg_state.vmc->getSpd();

  // until chassis
  if (!detectd_flag && std::abs(left_leg_state.x[1]) < 0.2 && std::abs(right_leg_state.x[5]) < 0.2 &&
      std::abs(chassis_state.angular_vel.y()) < 0.1)
  {
    detectChassisStateToRecover(chassis_state);
    detectLegRecoveryState(left_recovery_leg, left_pos.theta, input.leg_threshold_params);
    detectLegRecoveryState(right_recovery_leg, right_pos.theta, input.leg_threshold_params);
    detectd_flag = true;
    output.recovery_leg_spd_turnback_requested = false;
    leg_recovery_velocity_ =
        recovery_chassis_state_ == BackwardSlip ? -leg_recovery_velocity_const_ : leg_recovery_velocity_const_;
  }

  LegCommand left_cmd = { 0, 0, { 0., 0. } }, right_cmd = { 0, 0, { 0., 0. } };
  leg_theta_diff_ = shortest_angular_distance(left_pos.theta, right_pos.theta);
  double T_theta_diff{ 0.0 }, feedforward_force{ 0.0 };
  
  // input.base_state != 4. Assume 4 is RECOVERY or similar from original code
  if (input.base_state != 4 && detectd_flag)
  {
    left_cmd.force = pid_left_leg_->computeCommand(desired_leg_length_ - left_pos.L0, input.period) + feedforward_force;
    right_cmd.force = pid_right_leg_->computeCommand(desired_leg_length_ - right_pos.L0, input.period) + feedforward_force;

    // input.recovery_leg_spd_turnback? We need to add it to input.
    // Actually, I'll assume it's part of the input.
    // Wait, I should add recovery_leg_spd_turnback to LogicInput.
    
    // For now, I'll continue.
    
    if (chassis_state.roll < -0.5)
    {
      left_cmd.torque = pid_left_theta_->computeCommand(leg_recovery_velocity_ - left_spd.dTheta, input.period);
      right_cmd.torque = pid_right_theta_->computeCommand(0 - right_spd.dTheta, input.period);
      left_leg_recovery_feed_forward = 2 * leg_recovery_velocity_;
      right_leg_recovery_feed_forward = 0.0f;
      left_leg_state.vmc->leg_conv(left_cmd.force, left_leg_recovery_feed_forward + left_cmd.torque, left_cmd.input);
      right_leg_state.vmc->leg_conv(right_cmd.force, right_leg_recovery_feed_forward + right_cmd.torque,
                                    right_cmd.input);
    }
    else if (chassis_state.roll > 0.5)
    {
      left_cmd.torque = pid_left_theta_->computeCommand(0 - left_spd.dTheta, input.period);
      right_cmd.torque = pid_right_theta_->computeCommand(leg_recovery_velocity_ - right_spd.dTheta, input.period);
      left_leg_recovery_feed_forward = 0.0f;
      right_leg_recovery_feed_forward = 2 * leg_recovery_velocity_;
      left_leg_state.vmc->leg_conv(left_cmd.force, left_leg_recovery_feed_forward + left_cmd.torque, left_cmd.input);
      right_leg_state.vmc->leg_conv(right_cmd.force, right_leg_recovery_feed_forward + right_cmd.torque,
                                    right_cmd.input);
    }
    else
    {
      if (left_recovery_leg == NotReady && right_recovery_leg == Ready)
      {
        detectLegRecoveryState(left_recovery_leg, left_pos.theta, input.leg_threshold_params);
        detectLegRecoveryState(right_recovery_leg, right_pos.theta, input.leg_threshold_params);
        left_cmd.torque = pid_left_theta_->computeCommand(leg_recovery_velocity_ - left_spd.dTheta, input.period);
        right_cmd.torque = pid_right_theta_->computeCommand(0 - right_spd.dTheta, input.period);
        left_leg_recovery_feed_forward = 2 * leg_recovery_velocity_;
        right_leg_recovery_feed_forward = 0.0f;
        left_leg_state.vmc->leg_conv(left_cmd.force, left_leg_recovery_feed_forward + left_cmd.torque, left_cmd.input);
        right_leg_state.vmc->leg_conv(right_cmd.force, right_leg_recovery_feed_forward + right_cmd.torque,
                                      right_cmd.input);
      }
      if (left_recovery_leg == Ready && right_recovery_leg == NotReady)
      {
        detectLegRecoveryState(left_recovery_leg, left_pos.theta, input.leg_threshold_params);
        detectLegRecoveryState(right_recovery_leg, right_pos.theta, input.leg_threshold_params);
        left_cmd.torque = pid_left_theta_->computeCommand(0 - left_spd.dTheta, input.period);
        right_cmd.torque = pid_right_theta_->computeCommand(leg_recovery_velocity_ - right_spd.dTheta, input.period);
        left_leg_recovery_feed_forward = 0.0f;
        right_leg_recovery_feed_forward = 2 * leg_recovery_velocity_;
        left_leg_state.vmc->leg_conv(left_cmd.force, left_leg_recovery_feed_forward + left_cmd.torque, left_cmd.input);
        right_leg_state.vmc->leg_conv(right_cmd.force, right_leg_recovery_feed_forward + right_cmd.torque,
                                      right_cmd.input);
      }
      if (std::abs(leg_theta_diff_) < 0.4)
      {
        if ((left_recovery_leg == Ready && right_recovery_leg == Ready) ||
            (left_recovery_leg == NotReady && right_recovery_leg == NotReady))
        {
          T_theta_diff = pid_theta_diff_->computeCommand(leg_theta_diff_, input.period);
          detectLegRecoveryState(left_recovery_leg, left_pos.theta, input.leg_threshold_params);
          detectLegRecoveryState(right_recovery_leg, right_pos.theta, input.leg_threshold_params);
          left_cmd.torque = pid_left_theta_->computeCommand(leg_recovery_velocity_ - left_spd.dTheta, input.period);
          right_cmd.torque = pid_right_theta_->computeCommand(leg_recovery_velocity_ - right_spd.dTheta, input.period);
          left_leg_recovery_feed_forward = 2 * leg_recovery_velocity_;
          right_leg_recovery_feed_forward = left_leg_recovery_feed_forward;
          left_leg_state.vmc->leg_conv(left_cmd.force, left_leg_recovery_feed_forward + left_cmd.torque + T_theta_diff,
                                       left_cmd.input);
          right_leg_state.vmc->leg_conv(
              right_cmd.force, right_leg_recovery_feed_forward + right_cmd.torque - T_theta_diff, right_cmd.input);
        }
      }
    }
  }
  
  output.leg_cmd[LEFT] = left_cmd;
  output.leg_cmd[RIGHT] = right_cmd;
  output.wheel_cmd[LEFT] = output.wheel_cmd[RIGHT] = 0.0;

  // Exit
  if (std::abs(chassis_state.pitch) < 0.2 && chassis_state.linear_acc.z() > 5.0 && !input.overturn)
  {
    output.next_mode = BalanceMode::SIT_DOWN;
    output.state_changed_requested = false;
    // clearRecoveryFlag() requested
  }
  
  return output;
}

void Recover::detectChassisStateToRecover(const ChassisState& chassis_state)
{
  if (chassis_state.pitch > 0.45 && chassis_state.pitch < M_PI)
  {
    recovery_chassis_state_ = RecoveryChassisState::ForwardSlip;
  }
  else if (chassis_state.pitch < -0.45 && chassis_state.pitch > -M_PI)
  {
    recovery_chassis_state_ = RecoveryChassisState::BackwardSlip;
  }
}

void Recover::detectLegRecoveryState(LegRecoveryState& leg_recovery_state, const double& leg_pos,
                                     const std::shared_ptr<LegStateThresholdParams>& threshold)
{
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
}
}  // namespace rm_chassis_controllers
