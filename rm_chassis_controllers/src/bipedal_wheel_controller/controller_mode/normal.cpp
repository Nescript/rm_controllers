//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/normal.h"
#include "bipedal_wheel_controller/math_helpers.h"

namespace rm_chassis_controllers
{
Normal::Normal(PidInterface* pid_left_leg, PidInterface* pid_right_leg, PidInterface* pid_yaw_vel,
               PidInterface* pid_theta_diff, PidInterface* pid_roll, PidInterface* pid_wheel_vel_diff)
  : pid_left_leg_(pid_left_leg)
  , pid_right_leg_(pid_right_leg)
  , pid_yaw_vel_(pid_yaw_vel)
  , pid_theta_diff_(pid_theta_diff)
  , pid_roll_(pid_roll)
  , pid_wheel_vel_diff_(pid_wheel_vel_diff)
{
  leftSupportForceAveragePtr_ = std::make_shared<MovingAverageFilter<double>>(4);
  rightSupportForceAveragePtr_ = std::make_shared<MovingAverageFilter<double>>(4);
}

LogicOutput Normal::execute(const LogicInput& input)
{
  LogicOutput output;
  const auto& bias_params_ = input.bias_params;
  const auto& model_params_ = input.model_params;
  const auto& control_params_ = input.control_params;
  const auto& spring_params_ = input.spring_params;

  if (!input.state_changed)
  {
    output.clear_status_requested = true;
    jump_phase_ = JumpPhase::IDLE;
    pos_des_ = 0.0f;
    output.state_changed_requested = true;
  }
  const auto& chassis_state = input.chassis_state;
  const auto& left_leg_state = input.leg_state[LEFT];
  const auto& right_leg_state = input.leg_state[RIGHT];
  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();
  const auto& left_spd = left_leg_state.vmc->getSpd();
  const auto& right_spd = right_leg_state.vmc->getSpd();

  if (std::abs(left_leg_state.x[PITCH]) < 0.2 && (std::abs(left_leg_state.x[THETA] + right_leg_state.x[THETA]) / 2.0f) < 0.2)
  {
    protect_flag_ = false;
    if (!input.complete_stand)
    {
      output.complete_stand_requested = true;
    }
  }

  auto vel_cmd_ = input.instruction;
  double current_leg_length = (left_pos.L0 + right_pos.L0) / 2.0f;
  if (std::abs(chassis_state.x_vel) < 0.1f && std::abs(vel_cmd_.x_vel_cmd) < 0.01f)
  {
    output.move_flag_requested = false;
    if (x_offset_flag_)
    {
      x_offset_flag_ = false;
      pos_des_ =
          current_leg_length * std::sin(-(left_leg_state.x(THETA) + right_leg_state.x(THETA)) / 2.0f) + bias_params_->x;
    }
  }

  double friction_circle = chassis_state.x_vel * chassis_state.angular_vel.z();
  double friction_circle_alpha = std::abs(friction_circle) > 10.0f ? (10.0f / std::abs(friction_circle)) : 1.0f;
  // PID
  double T_yaw =
      pid_yaw_vel_->computeCommand(friction_circle_alpha * vel_cmd_.yaw_vel_cmd - chassis_state.angular_vel.z(), input.period);
  double theta_diff = right_pos.theta - left_pos.theta;
  double T_theta_diff = pid_theta_diff_->computeCommand(theta_diff, input.period);
  double F_roll = pid_roll_->computeCommand(0. - chassis_state.roll, input.period);
  // LQR
  Eigen::Matrix<double, 4, 12> coeffs_ = input.coeffs;
  Eigen::Matrix<double, 2, 6> k_left, k_right;
  k_left.setZero();
  k_right.setZero();

  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      k_left(i, j) = coeffs_(0, i + 2 * j) * std::pow(left_pos.L0, 3) + coeffs_(1, i + 2 * j) * std::pow(left_pos.L0, 2) +
                     coeffs_(2, i + 2 * j) * left_pos.L0 + coeffs_(3, i + 2 * j);
      k_right(i, j) = coeffs_(0, i + 2 * j) * std::pow(right_pos.L0, 3) + coeffs_(1, i + 2 * j) * std::pow(right_pos.L0, 2) +
                      coeffs_(2, i + 2 * j) * right_pos.L0 + coeffs_(3, i + 2 * j);
    }
  }

  Eigen::Matrix<double, CONTROL_DIM, 1> u_left, u_right;
  u_left.setZero();
  u_right.setZero();
  auto x_left = left_leg_state.x;
  auto x_right = right_leg_state.x;
  Eigen::Matrix<double, 6, 1> x_left_ref, x_right_ref;
  x_left_ref.setZero();
  x_right_ref.setZero();

  if (input.complete_stand)
  {
    x_left_ref(POS) = x_right_ref(POS) = pos_des_;
    if (input.base_state != 0) // Assume 0 is RAW
    {
      x_left_ref(VEL) = x_right_ref(VEL) = friction_circle_alpha * vel_cmd_.x_vel_cmd;
    }
    else
    {
      x_left_ref(VEL) = x_right_ref(VEL) = 0.0f;
    }
    if (protect_flag_)
    {
      x_left_ref(VEL) = x_right_ref(VEL) = 0.0f;
      leg_length_des = input.default_leg_length;
    }
    else
    {
      leg_length_des = vel_cmd_.leg_length_cmd;
    }
  }
  else
  {
    leg_length_des = input.default_leg_length;
  }
  if (input.base_state != 0)
  {
    if (!input.move_flag)
    {
      x_offset_flag_ = true;
      x_left(THETA) -= bias_params_->theta;
      x_right(THETA) -= bias_params_->theta;
    }
  }
  else
  {
    x_left(THETA) -= bias_params_->raw_theta;
    x_right(THETA) -= bias_params_->raw_theta;
    x_left(PITCH) -= bias_params_->raw_pitch;
    x_right(PITCH) -= bias_params_->raw_pitch;
  }

  x_left -= x_left_ref;
  x_right -= x_right_ref;

  clamp(x_left(VEL), -1.2, 1.2);
  clamp(x_right(VEL), -1.2, 1.2);

  const double k_pitch = -0.1f, b = 0.35f;
  double pitch_error_clamp = k_pitch * chassis_state.x_vel + b;
  clamp(x_left(PITCH), -pitch_error_clamp, pitch_error_clamp);
  clamp(x_right(PITCH), -pitch_error_clamp, pitch_error_clamp);
  clamp(x_left(THETA), -0.6, 0.6);
  clamp(x_right(THETA), -0.6, 0.6);

  u_left = k_left * (-x_left);
  u_right = k_right * (-x_right);

  // Compute leg thrust
  double wheel_vel_diff = std::abs(input.left_wheel_vel) - std::abs(input.right_wheel_vel);
  double gravity = model_params_->f_gravity;
  double left_spring_force = f_spring_force(left_pos.L0, left_leg_state.vmc->getL1(), left_leg_state.vmc->getL2(),
                                           spring_params_->f_spring, spring_params_->s2, spring_params_->s3, spring_params_->alpha_s);
  double right_spring_force = f_spring_force(right_pos.L0, right_leg_state.vmc->getL1(), right_leg_state.vmc->getL2(),
                                            spring_params_->f_spring, spring_params_->s2, spring_params_->s3, spring_params_->alpha_s);
  
  double F_inertia_left =
      model_params_->M * friction_circle * left_pos.L0 / input.chassis_geometry_params->wheel_track;
  double F_inertia_right =
      model_params_->M * friction_circle * right_pos.L0 / input.chassis_geometry_params->wheel_track;
  double F_pid_left{}, F_pid_right{}, T_wheel_diff{};
  Eigen::Matrix<double, 2, 1> F_leg;
  F_leg.setZero();
  
  // check jump
  if (jump_phase_ == JumpPhase::IDLE &&
      input.time - lastJumpTime_ > control_params_->jumpOverTime_ && vel_cmd_.jump_cmd)
  {
    jump_phase_ = JumpPhase::LEG_RETRACTION;
  }
  
  if (jump_phase_ == JumpPhase::IDLE)
  {
    static double last_left_length_des = leg_length_des, last_right_length_des = leg_length_des;
    double left_length_des_cur = input.complete_stand ? (0.8 * leg_length_des + 0.2 * last_left_length_des) :
                                                              input.default_leg_length;
    double right_length_des_cur = input.complete_stand ? (0.8 * leg_length_des + 0.2 * last_right_length_des) :
                                                               input.default_leg_length;
    last_left_length_des = left_length_des_cur;
    last_right_length_des = right_length_des_cur;
    F_pid_left = pid_left_leg_->computeCommand(left_length_des_cur - current_leg_length, input.period);
    F_pid_right = pid_right_leg_->computeCommand(right_length_des_cur - current_leg_length, input.period);
    F_pid_left = std::abs(F_pid_left) > 150 ? std::copysign(150.0, F_pid_left) : F_pid_left;
    F_pid_right = std::abs(F_pid_right) > 150 ? std::copysign(150.0, F_pid_right) : F_pid_right;
    F_leg[LEFT] = F_pid_left - F_inertia_left + gravity / std::cos(left_pos.theta) + F_roll - left_spring_force;
    F_leg[RIGHT] = F_pid_right + F_inertia_right + gravity / std::cos(right_pos.theta) - F_roll - right_spring_force;
    T_wheel_diff = input.base_state == 0 ? // Assume 0 is RAW
                       pid_wheel_vel_diff_->computeCommand(wheel_vel_diff, input.period) :
                       0.0f;
  }
  else
  {
    leg_length_des = jumpLengthDes[jump_phase_].second;
    double s_left = (left_pos.L0 - 0.12) / (0.35 - 0.11);
    double s_right = (right_pos.L0 - 0.12) / (0.35 - 0.11);
    switch (jump_phase_)
    {
      case JumpPhase::LEG_RETRACTION:
      {
        F_leg(LEFT) = pid_left_leg_->computeCommand(leg_length_des - current_leg_length, input.period) +
                      gravity / std::cos(left_pos.theta) + F_roll - left_spring_force;
        F_leg(RIGHT) = pid_right_leg_->computeCommand(leg_length_des - current_leg_length, input.period) +
                       gravity / std::cos(right_pos.theta) - F_roll - right_spring_force;
        if (current_leg_length < leg_length_des + 0.02f)
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
        F_leg(0) = 300 * (1 - 3 * std::pow(s_left, 2) + 2 * std::pow(s_left, 3)) + gravity;
        F_leg(1) = 300 * (1 - 3 * std::pow(s_right, 2) + 2 * std::pow(s_right, 3)) + gravity;
        if (current_leg_length > leg_length_des)
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
        double s_left_flip = 1 - s_left;
        double s_right_flip = 1 - s_left;
        F_leg(0) = -175 * (1 - 3 * std::pow(s_left_flip, 2) + 2 * std::pow(s_left_flip, 3)) - left_spring_force;
        F_leg(1) = -175 * (1 - 3 * std::pow(s_right_flip, 2) + 2 * std::pow(s_right_flip, 3)) - right_spring_force;

        if (current_leg_length < leg_length_des + 0.02f)
        {
          jumpTime_++;
        }
        if (jumpTime_ >= 100)
        {
          jumpTime_ = 0;
          jump_phase_ = JumpPhase::IDLE;
          lastJumpTime_ = input.time;
          output.jump_cmd_requested = false;
        }
        break;
    }
  }

  // Unstick detection
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k_left_unstick{}, k_right_unstick{};
  k_left_unstick.setZero();
  k_right_unstick.setZero();
  k_left_unstick.block<1, 2>(1, 0) = k_left.block<1, 2>(1, 0);
  k_right_unstick.block<1, 2>(1, 0) = k_right.block<1, 2>(1, 0);

  static bool last_left_unstick{ false }, last_right_unstick{ false };
  bool left_unstick{ false }, right_unstick{ false };
  if (jump_phase_ == JumpPhase::OFF_GROUND)
  {
    left_unstick = right_unstick = true;
  }
  else if (input.complete_stand && jump_phase_ != JumpPhase::LEG_RETRACTION &&
           input.base_state == 1) // Assume 1 is FOLLOW
  {
    left_unstick =
        unstickDetection(last_left_unstick ? F_pid_left : left_leg_state.vmc->getForceReal().F + left_spring_force,
                         u_left(LEG_Tp), left_spd.dL0, left_pos.L0, chassis_state.linear_acc.z(), model_params_,
                         left_leg_state.x, leftSupportForceAveragePtr_, input.period, input.time);
    right_unstick =
        unstickDetection(last_right_unstick ? F_pid_right : right_leg_state.vmc->getForceReal().F + right_spring_force,
                         u_right(LEG_Tp), right_spd.dL0, right_pos.L0, chassis_state.linear_acc.z(), model_params_,
                         right_leg_state.x, rightSupportForceAveragePtr_, input.period, input.time);
  }
  last_left_unstick = left_unstick;
  last_right_unstick = right_unstick;

  updateUnstick(left_unstick, right_unstick);
  bool unstick_flag = left_unstick && right_unstick;
  if ((input.complete_stand && unstick_flag && jump_phase_ != JumpPhase::LEG_RETRACTION) ||
      jump_phase_ == JumpPhase::OFF_GROUND)
  {
    u_left = k_left_unstick * (-x_left);
    u_right = k_right_unstick * (-x_right);
  }

  // Control
  double left_T[2], right_T[2];
  left_leg_state.vmc->leg_conv(F_leg[LEFT], u_left(LEG_Tp) + T_theta_diff, left_T);
  right_leg_state.vmc->leg_conv(F_leg[RIGHT], u_right(LEG_Tp) - T_theta_diff, right_T);
  double left_wheel_cmd = unstick_flag ? 0. : u_left(WHEEL_T) - T_yaw - T_wheel_diff;
  double right_wheel_cmd = unstick_flag ? 0. : u_right(WHEEL_T) + T_yaw - T_wheel_diff;
  
  output.leg_cmd[LEFT] = { F_leg[LEFT], u_left(LEG_Tp) + T_theta_diff, { left_T[0], left_T[1] } };
  output.leg_cmd[RIGHT] = { F_leg[RIGHT], u_right(LEG_Tp) - T_theta_diff, { right_T[0], right_T[1] } };
  output.wheel_cmd[LEFT] = left_wheel_cmd;
  output.wheel_cmd[RIGHT] = right_wheel_cmd;

  // Mode switching
  if (jump_phase_ == JumpPhase::IDLE && input.complete_stand && std::abs(x_left(0) + x_right(0)) / 2.0f > 0.50 &&
      std::abs(vel_cmd_.x_vel_cmd) > 0.1 && std::abs(x_left(3)) > 0.1 && ((left_pos.L0 + right_pos.L0) / 2.0f) > 0.30 &&
      leg_length_des > 0.30)
  {
    output.next_mode = BalanceMode::UPSTAIRS;
    output.state_changed_requested = false;
    output.jump_cmd_requested = false;
    output.complete_stand_requested = false;
    output.wheel_cmd[LEFT] = output.wheel_cmd[RIGHT] = 0;
  }

  if (leg_length_des < 0.22)
  {
    if ((std::abs(x_left(0)) > 0.6f || std::abs(x_right(0)) > 0.6f || std::abs(chassis_state.pitch) > 0.4 ||
         std::abs(chassis_state.roll) > 0.4) ||
        (std::abs(u_left(0)) + std::abs(u_right(0)) / 2.0f > 30.0f) ||
        (std::abs(chassis_state.x_vel - x_left_ref(VEL)) > 5.0f && std::abs(chassis_state.x_vel - vel_cmd_.x_vel_cmd) > 5.0f))
    {
      protect_flag_ = true;
      output.next_mode = BalanceMode::PROTECT;
      output.state_changed_requested = false;
      output.complete_stand_requested = false;
      output.jump_cmd_requested = false;
      output.wheel_cmd[LEFT] = output.wheel_cmd[RIGHT] = 0;
    }
  }
  if (std::abs(x_left(THETA)) > 1.0 || std::abs(x_right(THETA)) > 1.0 || std::abs(chassis_state.pitch) > 0.6 ||
      std::abs(chassis_state.roll) > 0.8 || input.overturn || std::abs(theta_diff) > 1.0 ||
      input.base_state == 4) // Assume 4 is FALLEN
  {
    output.next_mode = BalanceMode::SIT_DOWN;
    output.state_changed_requested = false;
    output.complete_stand_requested = false;
    output.jump_cmd_requested = false;
    output.wheel_cmd[LEFT] = output.wheel_cmd[RIGHT] = 0;
  }
  
  return output;
}

double Normal::calculateSupportForce(double F, double Tp, double leg_length, const double& leg_len_spd, double acc_z,
                                     Eigen::Matrix<double, STATE_DIM, 1> x,
                                     const std::shared_ptr<ModelParams>& model_params, double period)
{
  static double last_ddot_zM = acc_z - model_params->g, last_dot_theta = x(1),
                last_ddot_theta = 0.0, last_ddot_leg_len = 0.0, d_leg_len[3]{};
  if (period <= 0.0) period = 0.001;

  d_leg_len[0] = d_leg_len[1];
  d_leg_len[1] = d_leg_len[2];
  d_leg_len[2] = leg_len_spd;

  double P = F * std::cos(x(0)) + Tp * std::sin(x(0)) / leg_length;
  // lp filter
  double ddot_zM = 0.3 * (acc_z - model_params->g) + 0.7 * last_ddot_zM;
  double ddot_theta = 0.3 * ((x(1) - last_dot_theta) / period) + 0.7 * last_ddot_theta;
  double ddot_leg_len = 0.3 * (d_leg_len[2] - d_leg_len[0]) / (2 * period) + 0.7 * last_ddot_leg_len;
  last_dot_theta = x(1);
  last_ddot_theta = ddot_theta;
  last_ddot_leg_len = ddot_leg_len;
  double ddot_zw = ddot_zM - ddot_leg_len * std::cos(x(0)) + 2 * leg_len_spd * x(1) * std::sin(x(0)) +
                   +leg_length * (ddot_theta * std::sin(x(0)) + leg_length * x(1) * x(1) * std::cos(x(0)));
  double Fn = model_params->m_w * ddot_zw + model_params->m_w * model_params->g + P;

  return Fn;
}

bool Normal::unstickDetection(const double& F_leg, const double& Tp, const double& leg_len_spd,
                              const double& leg_length, const double& acc_z,
                              const std::shared_ptr<ModelParams>& model_params, Eigen::Matrix<double, STATE_DIM, 1> x,
                              const std::shared_ptr<MovingAverageFilter<double>>& supportForceAveragePtr,
                              double period, double time)
{
  static bool maybeChange = false, last_unstick_ = false;
  static double judgeTime;
  double Fn = calculateSupportForce(F_leg, Tp, leg_length, leg_len_spd, acc_z, x, model_params, period);
  supportForceAveragePtr->input(Fn);
  bool unstick_ = supportForceAveragePtr->output() < unstick_threshold;

  if (unstick_ != last_unstick_)
  {
    if (!maybeChange)
    {
      judgeTime = time;
      maybeChange = true;
    }
    else
    {
      if (time - judgeTime > 0.1)
      {
        last_unstick_ = unstick_;
      }
    }
  }
  else
  {
    maybeChange = false;
  }
  return unstick_;
}
}  // namespace rm_chassis_controllers
