#pragma once

#include "bipedal_wheel/core/fsm/fsm_context.h"
#include "bipedal_wheel/core/helper_functions.h"
#include <cmath>
#include <memory>
#include <Eigen/Dense>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class Normal
{
public:
  Normal()
  {
    leftSupportForceAveragePtr_ = std::make_shared<MovingAverageFilterType>(4);
    rightSupportForceAveragePtr_ = std::make_shared<MovingAverageFilterType>(4);
  }

  void execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx)
  {
    const auto& bias_params_ = ctx.config.bias;
    if (!ctx.balance_state_changed)
    {
      ctx.logger.info("[balance] Enter NORMAL");
      // Clear status: in legacy, it sets slip_flag_ to false, reset filters, etc.
      // We can reset our local variables here
      jump_phase_ = JumpPhase::IDLE;
      pos_des_ = 0.0;
      jump_cooldown_counter_ = ctx.config.control.jump_over_time;  // Allow immediate jump
      ctx.balance_state_changed = true;
    }

    const auto& chassis_state = ctx.sens_in.chassis_state;
    const auto& left_pos = ctx.left_vmc->getPos();
    const auto& right_pos = ctx.right_vmc->getPos();
    const auto& left_spd = ctx.left_vmc->getSpd();
    const auto& right_spd = ctx.right_vmc->getSpd();

    // Pitch and Theta validation for Complete Stand status
    // Note: State vector indices mappings:
    // left_leg_state.x: [theta, d_theta, pos, vel, pitch, d_pitch]
    // Since in Core, LQRStatus holds: theta, d_theta, x, dx, pitch, d_pitch
    // We can map these LQR states directly using LQRStatus estimated in Core!
    // Wait, let's verify if the state vector x is computed from estimator.
    // Yes! BipedalWheelCore has a member: lqr_status_
    // So left_leg_state.x corresponds to:
    // x[THETA] = left_pos.theta - pitch
    // x[D_THETA] = left_spd.dTheta - d_pitch
    // x[POS] = x (horizontal position)
    // x[VEL] = dx (horizontal velocity)
    // x[PITCH] = pitch
    // x[D_PITCH] = d_pitch
    // Let's compute these state vectors for left and right legs:
    Eigen::Matrix<double, STATE_DIM, 1> x_left, x_right;
    x_left[THETA] = left_pos.theta + chassis_state.pitch;
    x_left[D_THETA] = left_spd.dTheta + chassis_state.angular_vel.y();
    x_left[POS] = ctx.lqr_status.x;
    x_left[VEL] = ctx.lqr_status.dx;
    x_left[PITCH] = -chassis_state.pitch;
    x_left[D_PITCH] = -chassis_state.angular_vel.y();

    x_right[THETA] = right_pos.theta + chassis_state.pitch;
    x_right[D_THETA] = right_spd.dTheta + chassis_state.angular_vel.y();
    x_right[POS] = ctx.lqr_status.x;
    x_right[VEL] = ctx.lqr_status.dx;
    x_right[PITCH] = -chassis_state.pitch;
    x_right[D_PITCH] = -chassis_state.angular_vel.y();

    if (std::abs(x_left[PITCH]) < 0.2 && (std::abs(x_left[THETA] + x_right[THETA]) / 2.0) < 0.2)
    {
      protect_flag_ = false;
      if (!ctx.complete_stand)
      {
        ctx.complete_stand = true;
      }
    }

    double current_leg_length = (left_pos.L0 + right_pos.L0) / 2.0;
    if (std::abs(ctx.lqr_status.dx) < 0.1 && std::abs(ctx.cmd_in.vel_cmd.x()) < 0.01)
    {
      ctx.recovery_leg_spd_turnback = false;  // maps to setMoveFlag(false)
      if (x_offset_flag_)
      {
        x_offset_flag_ = false;
        pos_des_ = current_leg_length * std::sin(-(x_left[THETA] + x_right[THETA]) / 2.0) + bias_params_.x;
      }
    }

    double friction_circle = ctx.lqr_status.dx * chassis_state.angular_vel.z();
    double friction_circle_alpha = std::abs(friction_circle) > 10.0 ? (10.0 / std::abs(friction_circle)) : 1.0;

    // PID Commands
    double T_yaw = ctx.pid_yaw_vel->computeCommand(
        friction_circle_alpha * ctx.cmd_in.vel_cmd.z() - chassis_state.angular_vel.z(), ctx.dt);
    double theta_diff = right_pos.theta - left_pos.theta;
    double T_theta_diff = ctx.pid_theta_diff->computeCommand(theta_diff, ctx.dt);
    double F_roll = ctx.pid_roll->computeCommand(0.0 - chassis_state.roll, ctx.dt);

    // LQR Feedback Gain Interpolation using leg length
    if (ctx.cmd_in.coeffs == nullptr)
    {
      ctx.logger.error("coeffs is nullptr in state_normal!");
      return;
    }
    const Eigen::Matrix<double, 4, 12>& coeffs_ = *ctx.cmd_in.coeffs;
    Eigen::Matrix<double, 2, 6> k_left, k_right;
    k_left.setZero();
    k_right.setZero();

    for (int i = 0; i < 2; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        k_left(i, j) = coeffs_(0, i + 2 * j) * std::pow(left_pos.L0, 3) +
                       coeffs_(1, i + 2 * j) * std::pow(left_pos.L0, 2) + coeffs_(2, i + 2 * j) * left_pos.L0 +
                       coeffs_(3, i + 2 * j);
        k_right(i, j) = coeffs_(0, i + 2 * j) * std::pow(right_pos.L0, 3) +
                        coeffs_(1, i + 2 * j) * std::pow(right_pos.L0, 2) + coeffs_(2, i + 2 * j) * right_pos.L0 +
                        coeffs_(3, i + 2 * j);
      }
    }

    Eigen::Matrix<double, CONTROL_DIM, 1> u_left, u_right;
    u_left.setZero();
    u_right.setZero();

    Eigen::Matrix<double, 6, 1> x_left_ref, x_right_ref;
    x_left_ref.setZero();
    x_right_ref.setZero();

    if (ctx.complete_stand)
    {
      x_left_ref(POS) = x_right_ref(POS) = pos_des_;
      if (ctx.cmd_in.base_state != 1)  // RAW = 1
      {
        x_left_ref(VEL) = x_right_ref(VEL) = friction_circle_alpha * ctx.cmd_in.vel_cmd.x();
      }
      else
      {
        x_left_ref(VEL) = x_right_ref(VEL) = 0.0;
      }

      if (protect_flag_)
      {
        x_left_ref(VEL) = x_right_ref(VEL) = 0.0;
        leg_length_des = ctx.config.default_leg_length;
      }
      else
      {
        leg_length_des = ctx.cmd_in.leg_length_cmd;
      }
    }
    else
    {
      leg_length_des = ctx.config.default_leg_length;
    }

    if (ctx.cmd_in.base_state != 1)
    {
      if (!ctx.move_flag)
      {
        x_offset_flag_ = true;
        x_left(THETA) -= bias_params_.theta;
        x_right(THETA) -= bias_params_.theta;
      }
    }
    else
    {
      x_left(THETA) -= bias_params_.raw_theta;
      x_right(THETA) -= bias_params_.raw_theta;
      x_left(PITCH) -= bias_params_.raw_pitch;
      x_right(PITCH) -= bias_params_.raw_pitch;
    }

    x_left -= x_left_ref;
    x_right -= x_right_ref;

    clamp(x_left(VEL), -1.2, 1.2);
    clamp(x_right(VEL), -1.2, 1.2);

    const double k_pitch = -0.1, b = 0.35;
    double pitch_error_clamp = k_pitch * ctx.lqr_status.dx + b;
    clamp(x_left(PITCH), -pitch_error_clamp, pitch_error_clamp);
    clamp(x_right(PITCH), -pitch_error_clamp, pitch_error_clamp);
    clamp(x_left(THETA), -0.6, 0.6);
    clamp(x_right(THETA), -0.6, 0.6);

    // 1. Calculate unscaled wheel torques using unscaled LQR gains and clamped errors
    double T_L_unscaled = k_left(WHEEL_T, THETA) * (-x_left(THETA)) +
                          k_left(WHEEL_T, D_THETA) * (-x_left(D_THETA)) +
                          k_left(WHEEL_T, POS) * (-x_left(POS)) +
                          k_left(WHEEL_T, VEL) * (-x_left(VEL)) +
                          k_left(WHEEL_T, PITCH) * (-x_left(PITCH)) +
                          k_left(WHEEL_T, D_PITCH) * (-x_left(D_PITCH));
    double T_R_unscaled = k_right(WHEEL_T, THETA) * (-x_right(THETA)) +
                          k_right(WHEEL_T, D_THETA) * (-x_right(D_THETA)) +
                          k_right(WHEEL_T, POS) * (-x_right(POS)) +
                          k_right(WHEEL_T, VEL) * (-x_right(VEL)) +
                          k_right(WHEEL_T, PITCH) * (-x_right(PITCH)) +
                          k_right(WHEEL_T, D_PITCH) * (-x_right(D_PITCH));

    // 2. Wheel velocities
    double omega_L = ctx.sens_in.leg_state[bipedal_wheel_core::LEFT].wheel.vel;
    double omega_R = ctx.sens_in.leg_state[bipedal_wheel_core::RIGHT].wheel.vel;

    // 3. Power Observer estimation
    double effort_coeff = ctx.cmd_in.effort_coeff;
    double vel_coeff = ctx.cmd_in.vel_coeff;
    double power_offset = ctx.cmd_in.power_offset;
    double power_limit = ctx.cmd_in.power_limit;

    double P_const_offset = vel_coeff * (std::pow(omega_L, 2) + std::pow(omega_R, 2)) - power_offset;
    double P_est_unscaled = effort_coeff * (std::pow(T_L_unscaled, 2) + std::pow(T_R_unscaled, 2)) +
                            std::abs(T_L_unscaled * omega_L) + std::abs(T_R_unscaled * omega_R) +
                            P_const_offset;

    // 4. Calculate K gain scaling factor alpha
    double alpha = 1.0;
    if (P_est_unscaled > power_limit && power_limit > P_const_offset)
    {
      alpha = std::sqrt((power_limit - P_const_offset) / (P_est_unscaled - P_const_offset));
    }
    // Clamp alpha to [0.1, 1.0]
    if (alpha < 0.1) alpha = 0.1;
    if (alpha > 1.0) alpha = 1.0;

    // Apply alpha to position and velocity gains for the WHEEL_T row (index 0)
    k_left(WHEEL_T, POS) *= alpha;
    k_left(WHEEL_T, VEL) *= alpha;
    k_right(WHEEL_T, POS) *= alpha;
    k_right(WHEEL_T, VEL) *= alpha;

    // Save outputs to status
    ctx.lqr_status.k_scale = alpha;

    // 5. Compute scaled command
    u_left = k_left * (-x_left);
    u_right = k_right * (-x_right);

    // Compute actual power estimation with scaled torques
    double T_L_scaled = u_left(WHEEL_T);
    double T_R_scaled = u_right(WHEEL_T);
    double P_est_scaled = effort_coeff * (std::pow(T_L_scaled, 2) + std::pow(T_R_scaled, 2)) +
                          std::abs(T_L_scaled * omega_L) + std::abs(T_R_scaled * omega_R) +
                          P_const_offset;
    ctx.lqr_status.power = P_est_scaled;

    // Compute leg thrust forces
    double gravity = ctx.config.model_params.f_gravity;
    // Spring forces
    double left_spring_force =
        f_spring_force(left_pos.L0, ctx.left_vmc->getL1(), ctx.left_vmc->getL2(), ctx.config.spring);
    double right_spring_force =
        f_spring_force(right_pos.L0, ctx.right_vmc->getL1(), ctx.right_vmc->getL2(), ctx.config.spring);

    double F_inertia_left =
        ctx.config.model_params.M * friction_circle * left_pos.L0 / ctx.config.chassis_geometry.wheel_track;
    double F_inertia_right =
        ctx.config.model_params.M * friction_circle * right_pos.L0 / ctx.config.chassis_geometry.wheel_track;

    double F_pid_left = 0.0, F_pid_right = 0.0, T_wheel_diff = 0.0;
    Eigen::Matrix<double, 2, 1> F_leg = Eigen::Matrix<double, 2, 1>::Zero();

    // Check jump sequence
    jump_cooldown_counter_ += ctx.dt;
    if (jump_phase_ == JumpPhase::IDLE && jump_cooldown_counter_ > ctx.config.control.jump_over_time &&
        ctx.cmd_in.jump_cmd)
    {
      jump_phase_ = JumpPhase::LEG_RETRACTION;
      jump_cooldown_counter_ = 0.0;
      ctx.logger.info("[balance] Jump start");
    }

    if (jump_phase_ == JumpPhase::IDLE)
    {
      static double last_left_length_des = leg_length_des;
      static double last_right_length_des = leg_length_des;
      double left_length_des =
          ctx.complete_stand ? (0.8 * leg_length_des + 0.2 * last_left_length_des) : ctx.config.default_leg_length;
      double right_length_des =
          ctx.complete_stand ? (0.8 * leg_length_des + 0.2 * last_right_length_des) : ctx.config.default_leg_length;
      last_left_length_des = left_length_des;
      last_right_length_des = right_length_des;

      F_pid_left = ctx.pid_legs[LEFT]->computeCommand(left_length_des - current_leg_length, ctx.dt);
      F_pid_right = ctx.pid_legs[RIGHT]->computeCommand(right_length_des - current_leg_length, ctx.dt);
      F_pid_left = std::abs(F_pid_left) > 150 ? std::copysign(1.0, F_pid_left) * 150 : F_pid_left;
      F_pid_right = std::abs(F_pid_right) > 150 ? std::copysign(1.0, F_pid_right) * 150 : F_pid_right;

      F_leg[LEFT] = F_pid_left - F_inertia_left + gravity / std::cos(left_pos.theta) + F_roll - left_spring_force;
      F_leg[RIGHT] = F_pid_right + F_inertia_right + gravity / std::cos(right_pos.theta) - F_roll - right_spring_force;

      T_wheel_diff = (ctx.cmd_in.base_state == 1) ? ctx.pid_wheel_vel_diff->computeCommand(0.0, ctx.dt) :
                                                    0.0;  // wheel difference PID
    }
    else
    {
      // Jump states
      // In normal.cpp: jumpLengthDes contains target retraction, jump up, and off ground leg lengths
      // retraction: 0.11, jump_up: 0.34, off_ground: 0.11
      double target_jump_len = 0.12;
      if (jump_phase_ == JumpPhase::LEG_RETRACTION)
        target_jump_len = 0.11;
      else if (jump_phase_ == JumpPhase::JUMP_UP)
        target_jump_len = 0.34;
      else if (jump_phase_ == JumpPhase::OFF_GROUND)
        target_jump_len = 0.11;

      double s_left = (left_pos.L0 - 0.12) / (0.35 - 0.11);
      double s_right = (right_pos.L0 - 0.12) / (0.35 - 0.11);

      switch (jump_phase_)
      {
        case JumpPhase::IDLE:
          break;
        case JumpPhase::LEG_RETRACTION:
          ctx.logger.info("[balance] ENTER LEG_RETRACTION");
          F_leg(LEFT) = ctx.pid_legs[LEFT]->computeCommand(target_jump_len - current_leg_length, ctx.dt) +
                        gravity / std::cos(left_pos.theta) + F_roll - left_spring_force;
          F_leg(RIGHT) = ctx.pid_legs[RIGHT]->computeCommand(target_jump_len - current_leg_length, ctx.dt) +
                         gravity / std::cos(right_pos.theta) - F_roll - right_spring_force;
          if (current_leg_length < target_jump_len + 0.02)
          {
            jumpTime_++;
          }
          if (jumpTime_ >= 10)
          {
            jumpTime_ = 0;
            jump_phase_ = JumpPhase::JUMP_UP;
          }
          break;

        case JumpPhase::JUMP_UP:
          ctx.logger.info("[balance] ENTER JUMP_UP");
          F_leg(LEFT) = 300 * (1 - 3 * std::pow(s_left, 2) + 2 * std::pow(s_left, 3)) + gravity;
          F_leg(RIGHT) = 300 * (1 - 3 * std::pow(s_right, 2) + 2 * std::pow(s_right, 3)) + gravity;
          if (current_leg_length > target_jump_len)
          {
            jumpTime_++;
          }
          if (jumpTime_ >= 2)
          {
            jumpTime_ = 0;
            jump_phase_ = JumpPhase::OFF_GROUND;
          }
          break;

        case JumpPhase::OFF_GROUND:
          ctx.logger.info("[balance] ENTER OFF_GROUND");
          double s_left_flip = 1.0 - s_left;
          double s_right_flip = 1.0 - s_right;
          F_leg(LEFT) = -175 * (1 - 3 * std::pow(s_left_flip, 2) + 2 * std::pow(s_left_flip, 3)) - left_spring_force;
          F_leg(RIGHT) =
              -175 * (1 - 3 * std::pow(s_right_flip, 2) + 2 * std::pow(s_right_flip, 3)) - right_spring_force;

          if (current_leg_length < target_jump_len + 0.02)
          {
            jumpTime_++;
          }
          if (jumpTime_ >= 100)
          {
            jumpTime_ = 0;
            jump_phase_ = JumpPhase::IDLE;
            jump_cooldown_counter_ = 0.0;  // Reset cooldown
            ctx.logger.info("[balance] Jump end");
          }
          break;
      }
    }

    // Unstick detection
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_left_unstick, k_right_unstick;
    k_left_unstick.setZero();
    k_right_unstick.setZero();
    k_left_unstick.block<1, 2>(1, 0) = k_left.block<1, 2>(1, 0);
    k_right_unstick.block<1, 2>(1, 0) = k_right.block<1, 2>(1, 0);

    bool left_unstick = false, right_unstick = false;
    if (jump_phase_ == JumpPhase::OFF_GROUND)
    {
      left_unstick = right_unstick = true;
    }
    else if (ctx.complete_stand && jump_phase_ != JumpPhase::LEG_RETRACTION &&
             ctx.cmd_in.base_state == 2)  // FOLLOW = 2
    {
      left_unstick =
          unstickDetection_left(last_unstick_l_ ? F_pid_left : ctx.left_vmc->getForceReal().F + left_spring_force,
                                u_left(LEG_Tp), left_spd.dL0, left_pos.L0, chassis_state.linear_acc.z(),
                                ctx.config.model_params, x_left, ctx.dt);
      right_unstick =
          unstickDetection_right(last_unstick_r_ ? F_pid_right : ctx.right_vmc->getForceReal().F + right_spring_force,
                                 u_right(LEG_Tp), right_spd.dL0, right_pos.L0, chassis_state.linear_acc.z(),
                                 ctx.config.model_params, x_right, ctx.dt);
    }

    last_unstick_l_ = left_unstick;
    last_unstick_r_ = right_unstick;

    // Output LQR estimation data for packaging
    ctx.lqr_status.theta = (left_pos.theta + right_pos.theta) / 2.0;
    ctx.lqr_status.d_theta = (left_spd.dTheta + right_spd.dTheta) / 2.0;
    // (Actual visualization telemetry is published by ROS wrapper)

    bool unstick_flag = left_unstick && right_unstick;
    if ((ctx.complete_stand && unstick_flag && jump_phase_ != JumpPhase::LEG_RETRACTION) ||
        jump_phase_ == JumpPhase::OFF_GROUND)
    {
      u_left = k_left_unstick * (-x_left);
      u_right = k_right_unstick * (-x_right);
    }

    // Convert virtual forces to joint torques
    double left_T[2] = { 0.0 }, right_T[2] = { 0.0 };
    ctx.left_vmc->leg_conv(F_leg[LEFT], u_left(LEG_Tp) + T_theta_diff, left_T);
    ctx.right_vmc->leg_conv(F_leg[RIGHT], u_right(LEG_Tp) - T_theta_diff, right_T);

    double left_wheel_cmd = unstick_flag ? 0.0 : u_left(WHEEL_T) - T_yaw - T_wheel_diff;
    double right_wheel_cmd = unstick_flag ? 0.0 : u_right(WHEEL_T) + T_yaw - T_wheel_diff;

    // Fill outputs
    ctx.control_output.leg_cmd[LEFT].hip.effort = left_T[0];
    ctx.control_output.leg_cmd[LEFT].knee.effort = left_T[1];
    ctx.control_output.leg_cmd[LEFT].wheel.effort = left_wheel_cmd;

    ctx.control_output.leg_cmd[RIGHT].hip.effort = right_T[0];
    ctx.control_output.leg_cmd[RIGHT].knee.effort = right_T[1];
    ctx.control_output.leg_cmd[RIGHT].wheel.effort = right_wheel_cmd;

    // Transitions
    // Upstairs transition
    if (jump_phase_ == JumpPhase::IDLE && ctx.complete_stand && std::abs(x_left(0) + x_right(0)) / 2.0 > 0.50 &&
        std::abs(ctx.cmd_in.vel_cmd.x()) > 0.1 && std::abs(x_left(3)) > 0.1 &&
        ((left_pos.L0 + right_pos.L0) / 2.0) > 0.30 && leg_length_des > 0.30)
    {
      ctx.current_physical_state =
          RobotPhysicalState::STAND;  // upstairs transition in core FSM (or custom upstairs mode)
      // Note: Upstairs is maps to another FSM state
      ctx.balance_state_changed = false;
      ctx.complete_stand = false;
      ctx.logger.info("[balance] Exit NORMAL");
    }

    // Protection to protect mode
    if (leg_length_des < 0.22)
    {
      if ((std::abs(x_left(0)) > 0.6 || std::abs(x_right(0)) > 0.6 || std::abs(chassis_state.pitch) > 0.4 ||
           std::abs(chassis_state.roll) > 0.4) ||
          (std::abs(u_left(0)) + std::abs(u_right(0)) / 2.0 > 30.0) ||
          (std::abs(ctx.lqr_status.dx - x_left_ref(VEL)) > 5.0 &&
           std::abs(ctx.lqr_status.dx - ctx.cmd_in.vel_cmd.x()) > 5.0))
      {
        protect_flag_ = true;
        leg_length_des = ctx.config.default_leg_length;
        ctx.current_physical_state = RobotPhysicalState::HANGING;  // HANGING maps to Protect
        ctx.balance_state_changed = false;
        ctx.complete_stand = false;
        ctx.logger.info("[balance] Exit NORMAL");
      }
    }

    // Protection to sit_down
    if (std::abs(x_left(THETA)) > 1.0 || std::abs(x_right(THETA)) > 1.0 || std::abs(chassis_state.pitch) > 0.6 ||
        std::abs(chassis_state.roll) > 0.8 || ctx.overturn || std::abs(theta_diff) > 1.0 ||
        ctx.cmd_in.base_state == 3)  // FALLEN = 3
    {
      ctx.current_physical_state = RobotPhysicalState::FALLEN;  // FALLEN maps to SitDown
      ctx.balance_state_changed = false;
      ctx.complete_stand = false;
      ctx.logger.info("[balance] Exit NORMAL");
    }

    ctx.move_flag = false;
  }

private:
  double calculateSupportForce(double F, double Tp, double leg_length, double leg_len_spd, double acc_z,
                               const Eigen::Matrix<double, STATE_DIM, 1>& x, const LqrModelParams& model_params,
                               double dt, bool is_left)
  {
    double& last_ddot_zM = is_left ? last_ddot_zM_l_ : last_ddot_zM_r_;
    double& last_dot_theta = is_left ? last_dot_theta_l_ : last_dot_theta_r_;
    double& last_ddot_theta = is_left ? last_ddot_theta_l_ : last_ddot_theta_r_;
    double& last_ddot_leg_len = is_left ? last_ddot_leg_len_l_ : last_ddot_leg_len_r_;
    double* d_leg_len = is_left ? d_leg_len_l_ : d_leg_len_r_;

    d_leg_len[0] = d_leg_len[1];
    d_leg_len[1] = d_leg_len[2];
    d_leg_len[2] = leg_len_spd;

    double P = F * std::cos(x(0)) + Tp * std::sin(x(0)) / leg_length;
    double ddot_zM = 0.3 * (acc_z - model_params.g) + 0.7 * last_ddot_zM;
    double ddot_theta = 0.3 * ((x(1) - last_dot_theta) / dt) + 0.7 * last_ddot_theta;
    double ddot_leg_len = 0.3 * (d_leg_len[2] - d_leg_len[0]) / (2.0 * dt) + 0.7 * last_ddot_leg_len;

    last_dot_theta = x(1);
    last_ddot_theta = ddot_theta;
    last_ddot_leg_len = ddot_leg_len;
    last_ddot_zM = ddot_zM;

    double ddot_zw = ddot_zM - ddot_leg_len * std::cos(x(0)) + 2.0 * leg_len_spd * x(1) * std::sin(x(0)) +
                     leg_length * (ddot_theta * std::sin(x(0)) + leg_length * x(1) * x(1) * std::cos(x(0)));
    double Fn = model_params.m_w * ddot_zw + model_params.m_w * model_params.g + P;

    return Fn;
  }

  bool unstickDetection_left(double F_leg, double Tp, double leg_len_spd, double leg_length, double acc_z,
                             const LqrModelParams& model_params, const Eigen::Matrix<double, STATE_DIM, 1>& x,
                             double dt)
  {
    double Fn = calculateSupportForce(F_leg, Tp, leg_length, leg_len_spd, acc_z, x, model_params, dt, true);
    leftSupportForceAveragePtr_->input(Fn);
    bool unstick = leftSupportForceAveragePtr_->output() < unstick_threshold;

    if (unstick != last_unstick_l_)
    {
      if (!maybeChange_l_)
      {
        judge_time_counter_l_ = 0.0;
        maybeChange_l_ = true;
      }
      else
      {
        judge_time_counter_l_ += dt;
        if (judge_time_counter_l_ > 0.1)
        {
          last_unstick_l_ = unstick;
        }
      }
    }
    else
    {
      maybeChange_l_ = false;
    }
    return unstick;
  }

  bool unstickDetection_right(double F_leg, double Tp, double leg_len_spd, double leg_length, double acc_z,
                              const LqrModelParams& model_params, const Eigen::Matrix<double, STATE_DIM, 1>& x,
                              double dt)
  {
    double Fn = calculateSupportForce(F_leg, Tp, leg_length, leg_len_spd, acc_z, x, model_params, dt, false);
    rightSupportForceAveragePtr_->input(Fn);
    bool unstick = rightSupportForceAveragePtr_->output() < unstick_threshold;

    if (unstick != last_unstick_r_)
    {
      if (!maybeChange_r_)
      {
        judge_time_counter_r_ = 0.0;
        maybeChange_r_ = true;
      }
      else
      {
        judge_time_counter_r_ += dt;
        if (judge_time_counter_r_ > 0.1)
        {
          last_unstick_r_ = unstick;
        }
      }
    }
    else
    {
      maybeChange_r_ = false;
    }
    return unstick;
  }

  std::shared_ptr<MovingAverageFilterType> leftSupportForceAveragePtr_;
  std::shared_ptr<MovingAverageFilterType> rightSupportForceAveragePtr_;

  double pos_des_ = 0.0, leg_length_des = 0.2;
  double unstick_threshold = 15.0;
  JumpPhase jump_phase_ = JumpPhase::IDLE;
  int jumpTime_ = 0;
  bool x_offset_flag_ = false, protect_flag_ = false;
  double jump_cooldown_counter_ = 0.0;

  // Reentrant support force filter states
  double last_ddot_zM_l_ = 0.0, last_dot_theta_l_ = 0.0, last_ddot_theta_l_ = 0.0, last_ddot_leg_len_l_ = 0.0;
  double d_leg_len_l_[3] = { 0.0 };

  double last_ddot_zM_r_ = 0.0, last_dot_theta_r_ = 0.0, last_ddot_theta_r_ = 0.0, last_ddot_leg_len_r_ = 0.0;
  double d_leg_len_r_[3] = { 0.0 };

  bool maybeChange_l_ = false, last_unstick_l_ = false;
  double judge_time_counter_l_ = 0.0;

  bool maybeChange_r_ = false, last_unstick_r_ = false;
  double judge_time_counter_r_ = 0.0;
};

}  // namespace bipedal_wheel_core
