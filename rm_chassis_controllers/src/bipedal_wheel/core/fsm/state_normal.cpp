#include "bipedal_wheel/core/fsm/state_normal.h"
#include "bipedal_wheel/core/helper_functions.h"
#include "bipedal_wheel/core/fsm/jump_control_policy.h"
#include "bipedal_wheel/controllers.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::Normal()
{
  leftSupportForceAveragePtr_ = std::make_shared<MovingAverageFilterType>(4);
  rightSupportForceAveragePtr_ = std::make_shared<MovingAverageFilterType>(4);
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
void Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx)
{
  const auto& bias_params_ = ctx.config.bias;
  if (!ctx.balance_state_changed)
  {
    ctx.logger.info("[balance] Enter NORMAL");
    jump_phase_ = JumpPhase::IDLE;
    was_raw_ = raw_brake_ready_ = raw_brake_timed_out_ = false;
    raw_brake_elapsed_ = raw_brake_hold_elapsed_ = 0.0;
    pos_des_ = 0.0;
    ctx.lqr_status.x = 0.0;
    jump_cooldown_counter_ = ctx.config.control.jump_over_time;  // Allow immediate jump
    off_ground_timer_ = 0.0;
    jump_contact_detector_.reset();
    for (int side = 0; side < 2; ++side)
    {
      last_mpc_control_[side].setZero();
      mpc_failed_[side] = false;
      mpc_warn_elapsed_[side] = 0.0;
    }
    support_force_history_[LEFT] = SupportForceHistory{};
    support_force_history_[RIGHT] = SupportForceHistory{};
    leftSupportForceAveragePtr_->clear();
    rightSupportForceAveragePtr_->clear();
    ctx.balance_state_changed = true;
  }

  const auto& chassis_state = ctx.sens_in.chassis_state;
  const auto& left_pos = ctx.left_vmc->getPos();
  const auto& right_pos = ctx.right_vmc->getPos();
  const auto& left_spd = ctx.left_vmc->getSpd();
  const auto& right_spd = ctx.right_vmc->getSpd();

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
    if (!ctx.complete_stand)
    {
      ctx.complete_stand = true;
    }
  }

  double current_leg_length = (left_pos.L0 + right_pos.L0) / 2.0;
  const bool is_raw = ctx.cmd_in.base_state == 1;
  // Keep the unfiltered wheel/leg speed available throughout RAW_BRAKE, even
  // after its timeout. The filtered RAW velocity is masked for the spin A/B test.
  double raw_brake_velocity = std::numeric_limits<double>::quiet_NaN();
  if (is_raw)
  {
    const double wheel_radius = ctx.config.model_params.r;
    const double left_vel = (ctx.sens_in.leg_state[LEFT].wheel.vel + chassis_state.angular_vel.y() +
                             left_spd.dTheta) * wheel_radius +
                            left_pos.L0 * left_spd.dTheta * std::cos(left_pos.theta + chassis_state.pitch) +
                            left_spd.dL0 * std::sin(left_pos.theta + chassis_state.pitch);
    const double right_vel = (ctx.sens_in.leg_state[RIGHT].wheel.vel + chassis_state.angular_vel.y() +
                              right_spd.dTheta) * wheel_radius +
                             right_pos.L0 * right_spd.dTheta * std::cos(right_pos.theta + chassis_state.pitch) +
                             right_spd.dL0 * std::sin(right_pos.theta + chassis_state.pitch);
    raw_brake_velocity = 0.5 * (left_vel + right_vel);
  }
  if (!is_raw && std::abs(ctx.lqr_status.dx) < 0.1 && std::abs(ctx.cmd_in.vel_cmd.x()) < 0.01)
  {
    ctx.move_flag = false;
    if (x_offset_flag_)
    {
      x_offset_flag_ = false;
      pos_des_ = current_leg_length * std::sin(-(x_left[THETA] + x_right[THETA]) / 2.0) + bias_params_.x;
    }
  }

  if (is_raw && !was_raw_)
  {
    raw_brake_ready_ = raw_brake_timed_out_ = false;
    raw_brake_elapsed_ = raw_brake_hold_elapsed_ = 0.0;
    raw_brake_length_ = current_leg_length;
    // Keep FOLLOW's hold position at rest; if entering while moving, brake toward
    // the current position instead of jumping to FOLLOW's old offset target.
    if (!std::isfinite(raw_brake_velocity) ||
        std::abs(raw_brake_velocity) >= ctx.config.raw_brake.speed_threshold)
      pos_des_ = ctx.lqr_status.x;
    x_offset_flag_ = false;
    ctx.pid_yaw_vel->reset();
    ctx.pid_wheel_vel_diff->reset();
  }
  if (!is_raw && was_raw_)
  {
    raw_brake_ready_ = raw_brake_timed_out_ = false;
    raw_brake_elapsed_ = raw_brake_hold_elapsed_ = 0.0;
    ctx.pid_yaw_vel->reset();
    ctx.pid_wheel_vel_diff->reset();
  }
  was_raw_ = is_raw;
  if (is_raw && jump_phase_ != JumpPhase::IDLE)
  {
    raw_brake_ready_ = false;
    raw_brake_hold_elapsed_ = 0.0;
  }
  if (is_raw && !raw_brake_ready_ && !raw_brake_timed_out_ && ctx.complete_stand && jump_phase_ == JumpPhase::IDLE)
  {
    const auto& brake = ctx.config.raw_brake;
    raw_brake_elapsed_ += ctx.dt;
    if (std::isfinite(raw_brake_velocity) && std::abs(raw_brake_velocity) < brake.speed_threshold)
      raw_brake_hold_elapsed_ += ctx.dt;
    else
      raw_brake_hold_elapsed_ = 0.0;
    if (raw_brake_hold_elapsed_ >= brake.hold_time)
    {
      raw_brake_ready_ = true;
      ctx.pid_yaw_vel->reset();
    }
    else if (raw_brake_elapsed_ >= brake.timeout)
    {
      raw_brake_timed_out_ = true;  // Never release yaw based on timeout alone.
      ctx.logger.warn("[balance] RAW_BRAKE timeout: yaw remains disabled");
    }
  }
  ctx.raw_ready = is_raw && raw_brake_ready_ && jump_phase_ == JumpPhase::IDLE && ctx.complete_stand;
  // A zero dx also removes the brake feedback. Use measured speed in RAW_BRAKE
  // only; once released, RAW still feeds dx = 0 to prevent the spin drift.
  if (is_raw && !ctx.raw_ready && std::isfinite(raw_brake_velocity))
  {
    ctx.lqr_status.dx = raw_brake_velocity;
    x_left(VEL) = x_right(VEL) = raw_brake_velocity;
  }

  // Turning-circle and velocity-dependent yaw compensations are for FOLLOW, not in-place RAW rotation.
  const double friction_circle = is_raw ? 0.0 : ctx.lqr_status.dx * chassis_state.angular_vel.z();
  const double friction_circle_alpha = std::abs(friction_circle) > 10.0 ? (10.0 / std::abs(friction_circle)) : 1.0;

  // Keep yaw damping during braking; release the requested yaw speed only once stopped.
  const double yaw_ref = is_raw && !ctx.raw_ready ? 0.0 : friction_circle_alpha * ctx.cmd_in.vel_cmd.z();
  double T_yaw = ctx.pid_yaw_vel->computeCommand(yaw_ref - chassis_state.angular_vel.z(), ctx.dt);
  double theta_diff = right_pos.theta - left_pos.theta;
  double T_theta_diff = ctx.pid_theta_diff->computeCommand(theta_diff, ctx.dt);
  double F_roll = ctx.pid_roll->computeCommand(0.0 - chassis_state.roll, ctx.dt);

  // LQR Feedback Gain Interpolation using leg length
  if (ctx.cmd_in.coeffs == nullptr)
  {
    ctx.logger.error("LQR coefficients are missing in state_normal!");
    return;
  }
  Eigen::Matrix<double, 2, 6> k_left, k_right;

  Eigen::Matrix<double, CONTROL_DIM, 1> u_left, u_right;
  u_left.setZero();
  u_right.setZero();

  Eigen::Matrix<double, 6, 1> x_left_ref, x_right_ref;
  x_left_ref.setZero();
  x_right_ref.setZero();

  if (ctx.complete_stand)
  {
    if (!is_raw)
    {
      x_left_ref(VEL) = x_right_ref(VEL) = friction_circle_alpha * ctx.cmd_in.vel_cmd.x();
      double theta_bias = bias_params_.theta;
      x_left(THETA) -= theta_bias;
      x_right(THETA) -= theta_bias;
      if (!ctx.move_flag)
      {
        x_offset_flag_ = true;
      }
      else
      {
        pos_des_ = bias_params_.x;
      }
    }
    else
    {
      x_left_ref(VEL) = x_right_ref(VEL) = 0.0;  // RAW is in-place rotation, including RAW_BRAKE.
      x_left(THETA) -= bias_params_.raw_theta;
      x_right(THETA) -= bias_params_.raw_theta;
      x_left(PITCH) -= bias_params_.raw_pitch;
      x_right(PITCH) -= bias_params_.raw_pitch;
    }
    // Retain the longitudinal hold loop throughout braking AND spinning.
    x_left_ref(POS) = x_right_ref(POS) = pos_des_;

    leg_length_des = is_raw && !ctx.raw_ready ? raw_brake_length_ : ctx.cmd_in.leg_length_cmd;
  }
  else
  {
    leg_length_des = is_raw ? raw_brake_length_ : ctx.config.default_leg_length;
  }

  // FOLLOW rollover limits apply to the reference before forming the MPC error.
  // RAW uses a zero forward reference, with braking as its rotation interlock.
  const auto& rollover_params = ctx.config.rollover;
  double rollover_v_max = std::numeric_limits<double>::max();
  if (!is_raw && rollover_params.enabled)
  {
    const double cog_height = (left_pos.L0 + right_pos.L0) / 2.0 + ctx.config.chassis_geometry.chassis_height;
    if (cog_height > 0.0)
    {
      const double yaw_rate_abs = std::max(std::abs(chassis_state.angular_vel.z()), rollover_params.min_yaw_rate);
      const double tip_acc = rollover_params.margin * ctx.config.model_params.g *
                             ctx.config.chassis_geometry.wheel_track / (2.0 * cog_height);
      rollover_v_max = tip_acc / yaw_rate_abs;
    }
  }
  clamp(x_left_ref(VEL), -rollover_v_max, rollover_v_max);
  clamp(x_right_ref(VEL), -rollover_v_max, rollover_v_max);

  x_left -= x_left_ref;
  x_right -= x_right_ref;

  clamp(x_left(VEL), -1.1, 1.1);
  clamp(x_right(VEL), -1.1, 1.1);

  const double k_pitch = -0.1, b = 0.35;
  double pitch_error_clamp = k_pitch * ctx.lqr_status.dx + b;
  clamp(x_left(PITCH), -pitch_error_clamp, pitch_error_clamp);
  clamp(x_right(PITCH), -pitch_error_clamp, pitch_error_clamp);
  clamp(x_left(THETA), -0.6, 0.6);
  clamp(x_right(THETA), -0.6, 0.6);

  // YAML bounds are wheel joint torque and each leg joint motor torque.
  const auto& input_min = ctx.config.mpc.input_min;
  const auto& input_max = ctx.config.mpc.input_max;
  Eigen::Matrix<double, CONTROL_DIM, 1> mpc_u_min, mpc_u_max;
  mpc_u_min << input_min[WHEEL_T], -1e17;
  mpc_u_max << input_max[WHEEL_T], 1e17;

  // Compute leg thrust forces first (needed for Jacobian-based MPC input bounds)
  double gravity = ctx.config.model_params.f_gravity;
  // Spring forces
  double left_spring_force =
      f_spring_force(left_pos.L0);
  double right_spring_force =
      f_spring_force(right_pos.L0);

  double F_inertia_left =
      ctx.config.model_params.M * friction_circle * left_pos.L0 / ctx.config.chassis_geometry.wheel_track;
  double F_inertia_right =
      ctx.config.model_params.M * friction_circle * right_pos.L0 / ctx.config.chassis_geometry.wheel_track;

  double F_pid_left = 0.0, F_pid_right = 0.0, T_wheel_diff = 0.0;
  Eigen::Matrix<double, 2, 1> F_leg = Eigen::Matrix<double, 2, 1>::Zero();

  // Detect flight during push-off, before disabling the ground balance loop.
  // Continue sampling in OFF_GROUND for landing; update the filters only once per cycle.
  const bool checking_contact = jump_phase_ == JumpPhase::JUMP_UP || jump_phase_ == JumpPhase::OFF_GROUND;
  bool contact_left_unstick = false, contact_right_unstick = false;
  if (checking_contact)
  {
    contact_left_unstick = unstickDetection_left(
        ctx.left_vmc->getForceReal().F + left_spring_force, ctx.left_vmc->getForceReal().Tp, left_spd.dL0,
        left_pos.L0, chassis_state.linear_acc.z(), ctx.config.model_params, x_left, ctx.dt);
    contact_right_unstick = unstickDetection_right(
        ctx.right_vmc->getForceReal().F + right_spring_force, ctx.right_vmc->getForceReal().Tp, right_spd.dL0,
        right_pos.L0, chassis_state.linear_acc.z(), ctx.config.model_params, x_right, ctx.dt);
    const bool landed = jump_contact_detector_.update(leftSupportForceAveragePtr_->output(),
                                                      rightSupportForceAveragePtr_->output(), unstick_threshold,
                                                      ctx.dt);
    if (landed)
    {
      jump_phase_ = JumpPhase::IDLE;
      jumpTime_ = 0;
      jump_cooldown_counter_ = 0.0;
      ctx.control_output.clear_jump_cmd = true;
      ctx.pid_legs[LEFT]->reset();
      ctx.pid_legs[RIGHT]->reset();
      ctx.logger.info("[balance] Jump landed");
    }
  }

  // Check jump sequence
  jump_cooldown_counter_ += ctx.dt;
  if (jump_phase_ == JumpPhase::IDLE && jump_cooldown_counter_ > ctx.config.control.jump_over_time &&
      ctx.cmd_in.jump_cmd)
  {
    jump_phase_ = JumpPhase::LEG_RETRACTION;
    jumpTime_ = 0;
    jump_contact_detector_.reset();
    support_force_history_[LEFT] = SupportForceHistory{};
    support_force_history_[RIGHT] = SupportForceHistory{};
    leftSupportForceAveragePtr_->clear();
    rightSupportForceAveragePtr_->clear();
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

    // This is a common-mode wheel torque correction, not a yaw differential.
    // Apply it only after RAW_BRAKE has released spinning; keep braking independent of yaw direction.
    if (is_raw && ctx.raw_ready && std::abs(ctx.cmd_in.vel_cmd.z()) > 1e-6)
    {
      const double wheel_vel_diff = std::abs(ctx.sens_in.leg_state[LEFT].wheel.vel) -
                                    std::abs(ctx.sens_in.leg_state[RIGHT].wheel.vel);
      T_wheel_diff = std::copysign(1.0, ctx.cmd_in.vel_cmd.z()) *
                     ctx.pid_wheel_vel_diff->computeCommand(wheel_vel_diff, ctx.dt);
    }
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

    leg_length_des = target_jump_len;

    double s_left = (left_pos.L0 - 0.12) / (0.35 - 0.11);
    double s_right = (right_pos.L0 - 0.12) / (0.35 - 0.11);

    switch (jump_phase_)
    {
      case JumpPhase::IDLE:
        break;
      case JumpPhase::LEG_RETRACTION:
      {
        ctx.logger.info("[balance] ENTER LEG_RETRACTION");
        F_leg(LEFT) = ctx.pid_legs[LEFT]->computeCommand((target_jump_len - 0.02) - current_leg_length, ctx.dt) +
                      gravity / std::cos(left_pos.theta) + F_roll - left_spring_force;
        F_leg(RIGHT) = ctx.pid_legs[RIGHT]->computeCommand((target_jump_len - 0.02) - current_leg_length, ctx.dt) +
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
      }
      case JumpPhase::JUMP_UP:
      {
        ctx.logger.info("[balance] ENTER JUMP_UP");
        F_leg(LEFT) = ctx.config.control.jump_up_force * (1 - 3 * std::pow(s_left, 2) + 2 * std::pow(s_left, 3)) + gravity;
        F_leg(RIGHT) = ctx.config.control.jump_up_force * (1 - 3 * std::pow(s_right, 2) + 2 * std::pow(s_right, 3)) + gravity;
        if (current_leg_length > target_jump_len - 0.01)
          jumpTime_++;
        else
          jumpTime_ = 0;
        if (jumpTime_ >= 4)
        {
          jumpTime_ = 0;
          jump_phase_ = JumpPhase::OFF_GROUND;
          off_ground_timer_ = 0.0;
        }
        break;
      }
      case JumpPhase::OFF_GROUND:
      {
        ctx.logger.info("[balance] ENTER OFF_GROUND");
        off_ground_timer_ += ctx.dt;
        // Retract for 200 ms from phase entry, then extend until confirmed contact.
        if (off_ground_timer_ < 0.2)
        {
          double s_left_flip = 1.0 - s_left;
          double s_right_flip = 1.0 - s_right;
          F_leg(LEFT) = -ctx.config.control.off_ground_force * (1 - 3 * std::pow(s_left_flip, 2) + 2 * std::pow(s_left_flip, 3)) -
                        left_spring_force;
          F_leg(RIGHT) = -ctx.config.control.off_ground_force * (1 - 3 * std::pow(s_right_flip, 2) + 2 * std::pow(s_right_flip, 3)) -
                         right_spring_force;
        }
        else
        {
          F_leg(LEFT) = 100.0;
          F_leg(RIGHT) = 100.0;
        }

        // Do not resume ground feedback in mid-air if contact has not been confirmed.
        if (off_ground_timer_ > 1.5)
        {
          if (jump_contact_detector_.isAirborne())
          {
            F_leg[LEFT] = F_leg[RIGHT] = 0.0;
            if (off_ground_timer_ - ctx.dt <= 1.5)
              ctx.logger.warn("[balance] Jump flight timeout: waiting for landing with leg thrust off");
          }
          else
          {
            jump_phase_ = JumpPhase::IDLE;
            jumpTime_ = 0;
            jump_cooldown_counter_ = 0.0;
            ctx.control_output.clear_jump_cmd = true;
            ctx.pid_legs[LEFT]->reset();
            ctx.pid_legs[RIGHT]->reset();
            ctx.logger.warn("[balance] Jump flight not detected, fallback to IDLE");
          }
        }
        break;
      }
    }
  }

  // Select gains after phase transitions, including the first push-off cycle.
  const auto* jump_coeffs = ctx.cmd_in.jump_coeffs ? ctx.cmd_in.jump_coeffs : ctx.cmd_in.coeffs;
  const auto& coeffs_ = *(jump_phase_ == JumpPhase::IDLE ? ctx.cmd_in.coeffs : jump_coeffs);
  for (int i = 0; i < CONTROL_DIM; ++i)
  {
    for (int j = 0; j < STATE_DIM; ++j)
    {
      const int col = i + CONTROL_DIM * j;
      k_left(i, j) = ((coeffs_(0, col) * left_pos.L0 + coeffs_(1, col)) * left_pos.L0 + coeffs_(2, col)) *
                         left_pos.L0 +
                     coeffs_(3, col);
      k_right(i, j) = ((coeffs_(0, col) * right_pos.L0 + coeffs_(1, col)) * right_pos.L0 + coeffs_(2, col)) *
                          right_pos.L0 +
                      coeffs_(3, col);
    }
  }

  const bool flight_control = jump_phase_ == JumpPhase::OFF_GROUND;
  const double jump_theta_bias = ctx.complete_stand ? (is_raw ? bias_params_.raw_theta : bias_params_.theta) : 0.0;
  if (is_raw && jump_phase_ != JumpPhase::IDLE)
  {
    ctx.raw_ready = false;
    raw_brake_ready_ = false;
    raw_brake_hold_elapsed_ = 0.0;
    T_yaw = 0.0;
  }
  maskJumpFeedback(x_left, jump_phase_, left_pos.theta + chassis_state.pitch,
                   left_spd.dTheta + chassis_state.angular_vel.y(), jump_theta_bias);
  maskJumpFeedback(x_right, jump_phase_, right_pos.theta + chassis_state.pitch,
                   right_spd.dTheta + chassis_state.angular_vel.y(), jump_theta_bias);

  // Keep the jump on the LQR feedback used by rm_ws. The MPC model is tuned
  // for ground balance; joint effort is still limited after VMC conversion.
  const bool use_mpc = ctx.config.mpc.enabled && jump_phase_ == JumpPhase::IDLE;

  // MPC solve with joint-space torque constraints
  double left_tp_min, left_tp_max, right_tp_min, right_tp_max;
  const bool left_bounds_valid = ctx.left_vmc->leg_torque_bounds(F_leg[LEFT], T_theta_diff, input_min[LEG_Tp],
                                                                 input_max[LEG_Tp], left_tp_min, left_tp_max);
  const bool right_bounds_valid = ctx.right_vmc->leg_torque_bounds(F_leg[RIGHT], -T_theta_diff, input_min[LEG_Tp],
                                                                   input_max[LEG_Tp], right_tp_min, right_tp_max);
  mpc_u_min(LEG_Tp) = left_tp_min;
  mpc_u_max(LEG_Tp) = left_tp_max;
  const bool left_mpc_solved = use_mpc && left_bounds_valid &&
                               ctx.core->solveMpc(bipedal_wheel_core::LEFT, left_pos.L0, x_left,
                                                  Eigen::Matrix<double, STATE_DIM, 1>::Zero(), mpc_u_min, mpc_u_max,
                                                  -rollover_v_max, rollover_v_max, u_left);
  mpc_u_min(LEG_Tp) = right_tp_min;
  mpc_u_max(LEG_Tp) = right_tp_max;
  const bool right_mpc_solved = use_mpc && right_bounds_valid &&
                                ctx.core->solveMpc(bipedal_wheel_core::RIGHT, right_pos.L0, x_right,
                                                   Eigen::Matrix<double, STATE_DIM, 1>::Zero(), mpc_u_min, mpc_u_max,
                                                   -rollover_v_max, rollover_v_max, u_right);
  if (use_mpc)
  {
    // A failed ground solve must not silently switch controllers. Hold the last
    // successful MPC output (zero before the first success); motor limits below
    // still apply after VMC and yaw corrections.
    auto handle_result = [&](int side, bool solved, bool bounds_valid,
                             Eigen::Matrix<double, CONTROL_DIM, 1>& control) {
      if (solved)
      {
        last_mpc_control_[side] = control;
        mpc_failed_[side] = false;
        mpc_warn_elapsed_[side] = 0.0;
        return;
      }
      control = last_mpc_control_[side];
      mpc_warn_elapsed_[side] += ctx.dt;
      if (!mpc_failed_[side] || mpc_warn_elapsed_[side] >= 1.0)
      {
        ctx.logger.warn(std::string("[balance] MPC solve failed on ") + (side == LEFT ? "LEFT" : "RIGHT") +
                        (bounds_valid ? ": solver/cache failure" : ": invalid joint torque bounds") +
                        "; holding last successful MPC output, no LQR fallback");
        mpc_warn_elapsed_[side] = 0.0;
      }
      mpc_failed_[side] = true;
    };
    handle_result(LEFT, left_mpc_solved, left_bounds_valid, u_left);
    handle_result(RIGHT, right_mpc_solved, right_bounds_valid, u_right);
  }
  else
  {
    // Explicit LQR mode and jump feedback are independent of solve failures.
    u_left = k_left * (-x_left);
    u_right = k_right * (-x_right);
    for (int side = 0; side < 2; ++side)
    {
      last_mpc_control_[side].setZero();
      mpc_failed_[side] = false;
      mpc_warn_elapsed_[side] = 0.0;
    }
  }

  // Unstick detection
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_left_unstick, k_right_unstick;
  k_left_unstick.setZero();
  k_right_unstick.setZero();
  k_left_unstick.block<1, 2>(1, 0) = k_left.block<1, 2>(1, 0);
  k_right_unstick.block<1, 2>(1, 0) = k_right.block<1, 2>(1, 0);

  bool left_unstick = false, right_unstick = false;
  if (flight_control)
  {
    left_unstick = right_unstick = true;
  }
  else if (checking_contact)
  {
    // Reuse measurements after landing; keep ground feedback until flight is confirmed.
    if (jump_phase_ == JumpPhase::IDLE)
    {
      left_unstick = contact_left_unstick;
      right_unstick = contact_right_unstick;
    }
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

  // JUMP_UP keeps ground feedback until the length target is reached, but its
  // contact measurements still belong in the public unstick status.
  const bool reported_left_unstick = left_unstick || (jump_phase_ == JumpPhase::JUMP_UP && contact_left_unstick);
  const bool reported_right_unstick = right_unstick || (jump_phase_ == JumpPhase::JUMP_UP && contact_right_unstick);
  bool unstick[2] = { reported_left_unstick, reported_right_unstick };
  Eigen::Vector2d F_leg_vec(F_leg[LEFT], F_leg[RIGHT]);
  bool unstick_flag = left_unstick && right_unstick;
  ctx.logger.publishLqrStatus(-x_left, -x_right, x_left_ref, x_right_ref, u_left, u_right, F_leg_vec, unstick);
  // Report measured contact loss as in rm_ws, including intentional flight.
  ctx.logger.publishUnstick(reported_left_unstick && reported_right_unstick);
  if ((ctx.complete_stand && unstick_flag && jump_phase_ != JumpPhase::LEG_RETRACTION) || flight_control)
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

  // Held MPC outputs and extra yaw/differential commands also need motor-side limits.
  auto limit_effort = [](double effort, double minimum, double maximum) {
    return std::isfinite(effort) ? std::clamp(effort, minimum, maximum) : 0.0;
  };
  const double jump_limit = ctx.config.control.jump_joint_effort_limit;
  const double joint_min = jump_phase_ != JumpPhase::IDLE && jump_limit > 0.0 ? -jump_limit : input_min[LEG_Tp];
  const double joint_max = jump_phase_ != JumpPhase::IDLE && jump_limit > 0.0 ? jump_limit : input_max[LEG_Tp];
  for (double& effort : left_T)
    effort = limit_effort(effort, joint_min, joint_max);
  for (double& effort : right_T)
    effort = limit_effort(effort, joint_min, joint_max);
  left_wheel_cmd = limit_effort(left_wheel_cmd, input_min[WHEEL_T], input_max[WHEEL_T]);
  right_wheel_cmd = limit_effort(right_wheel_cmd, input_min[WHEEL_T], input_max[WHEEL_T]);

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

  // Fall detection to sit_down
  if (std::abs(x_left(THETA)) > 1.0 || std::abs(x_right(THETA)) > 1.0 || std::abs(chassis_state.pitch) > 0.6 ||
      std::abs(chassis_state.roll) > 0.8 || ctx.overturn || std::abs(theta_diff) > 1.0 ||
      ctx.cmd_in.base_state == 3)  // FALLEN = 3
  {
    ctx.current_physical_state = RobotPhysicalState::FALLEN;  // FALLEN maps to SitDown
    ctx.balance_state_changed = false;
    ctx.complete_stand = false;
    ctx.logger.info("[balance] Exit NORMAL");
  }
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
double Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::calculateSupportForce(
    double F, double Tp, double leg_length, double leg_len_spd, double acc_z,
    const Eigen::Matrix<double, STATE_DIM, 1>& x, const LqrModelParams& model_params, double dt, bool is_left)
{
  auto& history = support_force_history_[is_left ? LEFT : RIGHT];
  const double P = F * std::cos(x(0)) + Tp * std::sin(x(0)) / leg_length;
  if (!(dt > 0.0))
    return model_params.m_w * model_params.g + P;

  if (!history.initialized)
  {
    history.initialized = true;
    history.ddot_zM = acc_z - model_params.g;
    history.dot_theta = x(1);
    for (double& speed : history.leg_len_spd)
      speed = leg_len_spd;
  }

  auto& d_leg_len = history.leg_len_spd;
  d_leg_len[0] = d_leg_len[1];
  d_leg_len[1] = d_leg_len[2];
  d_leg_len[2] = leg_len_spd;

  double ddot_zM = 0.3 * (acc_z - model_params.g) + 0.7 * history.ddot_zM;
  double ddot_theta = 0.3 * ((x(1) - history.dot_theta) / dt) + 0.7 * history.ddot_theta;
  double ddot_leg_len = 0.3 * (d_leg_len[2] - d_leg_len[0]) / (2.0 * dt) + 0.7 * history.ddot_leg_len;

  history.ddot_zM = ddot_zM;
  history.dot_theta = x(1);
  history.ddot_theta = ddot_theta;
  history.ddot_leg_len = ddot_leg_len;

  double ddot_zw = ddot_zM - ddot_leg_len * std::cos(x(0)) + 2.0 * leg_len_spd * x(1) * std::sin(x(0)) +
                   leg_length * (ddot_theta * std::sin(x(0)) + leg_length * x(1) * x(1) * std::cos(x(0)));
  double Fn = model_params.m_w * ddot_zw + model_params.m_w * model_params.g + P;

  return Fn;
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
bool Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::unstickDetection_left(
    double F_leg, double Tp, double leg_len_spd, double leg_length, double acc_z,
    const LqrModelParams& model_params, const Eigen::Matrix<double, STATE_DIM, 1>& x, double dt)
{
  double Fn = calculateSupportForce(F_leg, Tp, leg_length, leg_len_spd, acc_z, x, model_params, dt, true);
  leftSupportForceAveragePtr_->input(Fn);
  return leftSupportForceAveragePtr_->output() < unstick_threshold;
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
bool Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::unstickDetection_right(
    double F_leg, double Tp, double leg_len_spd, double leg_length, double acc_z,
    const LqrModelParams& model_params, const Eigen::Matrix<double, STATE_DIM, 1>& x, double dt)
{
  double Fn = calculateSupportForce(F_leg, Tp, leg_length, leg_len_spd, acc_z, x, model_params, dt, false);
  rightSupportForceAveragePtr_->input(Fn);
  return rightSupportForceAveragePtr_->output() < unstick_threshold;
}

// Explicit instantiation for the ROS controller's injected types.
template class Normal<rm_chassis_controllers::RosPidWrapper, rm_chassis_controllers::RosLoggerWrapper,
                      RampFilter<double>, MovingAverageFilter<double>>;

}  // namespace bipedal_wheel_core
