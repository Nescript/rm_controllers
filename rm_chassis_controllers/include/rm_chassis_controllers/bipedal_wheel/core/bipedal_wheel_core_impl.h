#pragma once

#include "bipedal_wheel/core/bipedal_wheel_core.h"
#include <cmath>
#include <vector>
#include <variant>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
bool BipedalWheelCore<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::init(
    const bipedal_wheel_core::ControllerParams& config, LoggerType& logger, const std::vector<PidType*>& pid_legs,
    const std::vector<PidType*>& pid_legs_stand_up, const std::vector<PidType*>& pid_thetas,
    const std::vector<PidType*>& pid_wheels, PidType* pid_yaw_vel, PidType* pid_theta_diff, PidType* pid_roll,
    PidType* pid_wheel_vel_diff)
{
  const auto& params = config.model_params;

  if (params.m_w <= 0.0 || params.m_p <= 0.0 || params.M <= 0.0 || params.i_w <= 0.0 || params.i_p <= 0.0 ||
      params.i_m <= 0.0 || params.r <= 0.0 || params.l <= 0.0 || params.g <= 0.0 || params.l1 <= 0.0 ||
      params.l2 <= 0.0)
  {
    logger.info("[BipedalWheelCore] Parameter check failed: Physical parameters must be greater than 0.");
    return false;
  }

  const auto& brake = config.raw_brake;
  if (!std::isfinite(brake.speed_threshold) || brake.speed_threshold <= 0.0 ||
      !std::isfinite(brake.hold_time) || brake.hold_time <= 0.0 ||
      !std::isfinite(brake.timeout) || brake.timeout <= brake.hold_time)
  {
    logger.error("[BipedalWheelCore] Invalid RAW brake configuration");
    return false;
  }

  const auto& input_min = config.mpc.input_min;
  const auto& input_max = config.mpc.input_max;
  if (input_min.size() != CONTROL_DIM || input_max.size() != CONTROL_DIM)
  {
    logger.error("[BipedalWheelCore] mpc_input_min/max must each have two values");
    return false;
  }
  for (int input = 0; input < CONTROL_DIM; ++input)
  {
    if (!std::isfinite(input_min[input]) || !std::isfinite(input_max[input]) ||
        input_min[input] >= 0.0 || input_max[input] <= 0.0)
    {
      logger.error("[BipedalWheelCore] Invalid mpc_input_min/max torque bounds");
      return false;
    }
  }

  // Construct left and right VMC solvers
  left_vmc_ = std::make_unique<VMC>(params.l1, params.l2, params.l3, params.l4, params.l5, params.five_link);
  right_vmc_ = std::make_unique<VMC>(params.l1, params.l2, params.l3, params.l4, params.l5, params.five_link);

  config_ = config;
  logger_ = &logger;
  pid_legs_ = pid_legs;
  pid_legs_stand_up_ = pid_legs_stand_up;
  pid_thetas_ = pid_thetas;
  pid_wheels_ = pid_wheels;
  pid_yaw_vel_ = pid_yaw_vel;
  pid_theta_diff_ = pid_theta_diff;
  pid_roll_ = pid_roll;
  pid_wheel_vel_diff_ = pid_wheel_vel_diff;

  // Initialize variant state to SitDown
  FSM_variant_ = SitDown<PidType, LoggerType, RampFilterType, MovingAverageFilterType>();

  kalman_filter_.init(0.0);
  last_linear_acc_base_x_ = 0.0;
  slip_flag_ = false;

  // Initialize continuous 1D LPV MPC solvers.
  if (config_.mpc.enabled)
  {
    if (!std::isfinite(config_.mpc.lpv_nominal_leg_length) || config_.mpc.lpv_nominal_leg_length <= 0.0)
    {
      logger.error("[BipedalWheelCore] Invalid mpc_lpv_nominal_leg_length");
      return false;
    }

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> continuous_a{};
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> continuous_b{};
    generateAB(config_.model_params, continuous_a, continuous_b, config_.mpc.lpv_nominal_leg_length);

    mpc_left_solver_ = std::make_unique<TinyMpcController>();
    mpc_right_solver_ = std::make_unique<TinyMpcController>();
    if (!mpc_left_solver_->init(continuous_a, continuous_b, config_.q, config_.r, config_.mpc) ||
        !mpc_right_solver_->init(continuous_a, continuous_b, config_.q, config_.r, config_.mpc))
    {
      logger.error("[BipedalWheelCore] Failed to initialize TinyMPC");
      return false;
    }
    if (!mpc_lpv_1d_.init(config_.model_params, config_.q, config_.r, config_.mpc))
    {
      logger.error("[BipedalWheelCore] Failed to initialize 1D LPV MPC cache");
      return false;
    }
    logger.info("[BipedalWheelCore] 1D LPV TinyMPC initialized");
  }

  return true;
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
ControlOutput BipedalWheelCore<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::update(
    double dt, const bipedal_wheel_core::SensorMeasurements& sens_in,
    const bipedal_wheel_core::ControllerCommands& cmd_in)
{
  // 1. Update estimation
  updateEstimation(dt, sens_in, cmd_in);

  // 2. Perform state machine switching if balance_state_changed_ is false
  if (!balance_state_changed_)
  {
    if (robot_mode_ == RobotPhysicalState::FALLEN)
    {
      FSM_variant_ = SitDown<PidType, LoggerType, RampFilterType, MovingAverageFilterType>();
    }
    else if (robot_mode_ == RobotPhysicalState::GETTING_UP)
    {
      if (overturn_)
      {
        FSM_variant_ = Recover<PidType, LoggerType, RampFilterType, MovingAverageFilterType>();
      }
      else
      {
        FSM_variant_ = StandUp<PidType, LoggerType, RampFilterType, MovingAverageFilterType>();
      }
    }
    else if (robot_mode_ == RobotPhysicalState::STAND)
    {
      if (std::holds_alternative<Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>>(FSM_variant_))
      {
        FSM_variant_ = Upstairs<PidType, LoggerType, RampFilterType, MovingAverageFilterType>();
      }
      else
      {
        FSM_variant_ = Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>();
      }
    }
  }

  // 3. Dynamically set the active leg PIDs
  if (std::holds_alternative<Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>>(FSM_variant_))
  {
    active_pid_legs_ = pid_legs_;
  }
  else
  {
    active_pid_legs_ = pid_legs_stand_up_;
  }

  // 4. Create context for state execution
  raw_ready_ = false;  // Non-NORMAL states must not acknowledge RAW rotation.
  ControlOutput control_output{};
  FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType> ctx{ config_,
                                                                                sens_in,
                                                                                cmd_in,
                                                                                control_output,
                                                                                lqr_status_,
                                                                                robot_mode_,
                                                                                complete_stand_,
                                                                                overturn_,
                                                                                move_flag_,
                                                                                balance_state_changed_,
                                                                                recovery_leg_spd_turnback_,
                                                                                raw_ready_,
                                                                                left_vmc_.get(),
                                                                                right_vmc_.get(),
                                                                                dt,
                                                                                *logger_,
                                                                                active_pid_legs_,
                                                                                pid_thetas_,
                                                                                pid_wheels_,
                                                                                pid_yaw_vel_,
                                                                                pid_theta_diff_,
                                                                                pid_roll_,
                                                                                pid_wheel_vel_diff_,
                                                                                this };

  // 5. Run active state
  std::visit([&ctx](auto& state) { state.execute(ctx); }, FSM_variant_);
  if (!std::holds_alternative<Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>>(FSM_variant_))
  {
    logger_->publishUnstick(false);
  }

  // 6. Update core's member variables from the context
  robot_mode_ = ctx.current_physical_state;
  complete_stand_ = ctx.complete_stand;
  overturn_ = ctx.overturn;
  move_flag_ = ctx.move_flag;
  balance_state_changed_ = ctx.balance_state_changed;
  recovery_leg_spd_turnback_ = ctx.recovery_leg_spd_turnback;

  return control_output;
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
void BipedalWheelCore<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::reset()
{
  lqr_status_ = bipedal_wheel_core::LQRStatus{};
  robot_mode_ = bipedal_wheel_core::RobotPhysicalState::FALLEN;
  complete_stand_ = false;
  overturn_ = false;
  move_flag_ = false;
  balance_state_changed_ = false;
  recovery_leg_spd_turnback_ = false;
  raw_ready_ = false;
  last_linear_acc_base_x_ = 0.0;
  slip_flag_ = false;
  kalman_filter_.init(0.0);

  if (left_vmc_)
  {
    *left_vmc_ = VMC(config_.model_params.l1, config_.model_params.l2, config_.model_params.l3,
                     config_.model_params.l4, config_.model_params.l5, config_.model_params.five_link);
  }
  if (right_vmc_)
  {
    *right_vmc_ = VMC(config_.model_params.l1, config_.model_params.l2, config_.model_params.l3,
                      config_.model_params.l4, config_.model_params.l5, config_.model_params.five_link);
  }
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
void BipedalWheelCore<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::updateEstimation(
    double dt, const bipedal_wheel_core::SensorMeasurements& sens_in,
    const bipedal_wheel_core::ControllerCommands& cmd_in)
{
  // 1. Calculate left leg VMC kinematics
  if (left_vmc_)
  {
    left_vmc_->calc_jacobian(sens_in.leg_state[bipedal_wheel_core::LEFT].hip.position,
                             sens_in.leg_state[bipedal_wheel_core::LEFT].knee.position);
    left_vmc_->leg_pos(sens_in.leg_state[bipedal_wheel_core::LEFT].hip.position,
                       sens_in.leg_state[bipedal_wheel_core::LEFT].knee.position);
    left_vmc_->leg_spd(sens_in.leg_state[bipedal_wheel_core::LEFT].hip.vel,
                       sens_in.leg_state[bipedal_wheel_core::LEFT].knee.vel);
    left_vmc_->leg_conv_t(sens_in.leg_state[bipedal_wheel_core::LEFT].hip.effort,
                          sens_in.leg_state[bipedal_wheel_core::LEFT].knee.effort);
  }

  // 2. Calculate right leg VMC kinematics
  if (right_vmc_)
  {
    right_vmc_->calc_jacobian(sens_in.leg_state[bipedal_wheel_core::RIGHT].hip.position,
                              sens_in.leg_state[bipedal_wheel_core::RIGHT].knee.position);
    right_vmc_->leg_pos(sens_in.leg_state[bipedal_wheel_core::RIGHT].hip.position,
                        sens_in.leg_state[bipedal_wheel_core::RIGHT].knee.position);
    right_vmc_->leg_spd(sens_in.leg_state[bipedal_wheel_core::RIGHT].hip.vel,
                        sens_in.leg_state[bipedal_wheel_core::RIGHT].knee.vel);
    right_vmc_->leg_conv_t(sens_in.leg_state[bipedal_wheel_core::RIGHT].hip.effort,
                           sens_in.leg_state[bipedal_wheel_core::RIGHT].knee.effort);
  }

  // 3. Absolute velocity calculation for base estimation (from wheels + virtual legs)
  double wheel_radius = config_.model_params.r;
  double pitch = sens_in.chassis_state.pitch;

  const auto& left_pos = left_vmc_->getPos();
  const auto& left_spd = left_vmc_->getSpd();
  const auto& right_pos = right_vmc_->getPos();
  const auto& right_spd = right_vmc_->getSpd();

  double leftWheelVel = (sens_in.leg_state[bipedal_wheel_core::LEFT].wheel.vel + sens_in.chassis_state.angular_vel.y() +
                         left_spd.dTheta) *
                        wheel_radius;
  double rightWheelVel = (sens_in.leg_state[bipedal_wheel_core::RIGHT].wheel.vel +
                          sens_in.chassis_state.angular_vel.y() + right_spd.dTheta) *
                         wheel_radius;

  double leftWheelVelAbsolute = leftWheelVel + left_pos.L0 * left_spd.dTheta * std::cos(left_pos.theta + pitch) +
                                left_spd.dL0 * std::sin(left_pos.theta + pitch);
  double rightWheelVelAbsolute = rightWheelVel + right_pos.L0 * right_spd.dTheta * std::cos(right_pos.theta + pitch) +
                                 right_spd.dL0 * std::sin(right_pos.theta + pitch);

  double wheel_vel_aver = (leftWheelVelAbsolute + rightWheelVelAbsolute) / 2.0;

  // 4. Run Kalman Filter for base horizontal velocity & acceleration
  double R_wheel = 200.0;
  double slip_R_wheel = 2.0 * R_wheel;

  Eigen::Matrix2d R;
  R << (slip_flag_ ? slip_R_wheel : R_wheel), 0.0, 0.0, 200.0;

  Eigen::Vector2d z;
  z << wheel_vel_aver, 0.2 * last_linear_acc_base_x_ + 0.8 * sens_in.chassis_state.linear_acc.x();
  last_linear_acc_base_x_ = z(1);

  kalman_filter_.predict(dt);
  kalman_filter_.update(z, R);

  Eigen::Vector2d x_hat = kalman_filter_.getState();
  slip_flag_ = std::abs(x_hat(0) - wheel_vel_aver) > 3.0;

  // 5. Update estimated state
  overturn_ = cmd_in.overturn;

  // A/B experiment: do not feed the slow Kalman velocity into RAW balance.
  // RAW_BRAKE checks unfiltered wheel/leg velocity separately before allowing yaw.
  lqr_status_.dx = cmd_in.base_state == 1 ? 0.0 : x_hat(0);

  // A/B experiment: keep measured RAW speed for braking, but remove accumulated
  // longitudinal position feedback in RAW. FOLLOW estimation is unchanged.
  if (cmd_in.base_state == 1)
  {
    lqr_status_.x = 0.0;
  }
  else if (std::abs(lqr_status_.dx) <= 0.5 && std::abs(cmd_in.vel_cmd.x()) <= 0.01)
  {
    lqr_status_.x += lqr_status_.dx * dt;
  }
  else
  {
    move_flag_ = true;  // resets position and indicates moving state
    lqr_status_.x = 0.0;
  }
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
FsmState BipedalWheelCore<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::getFsmState() const
{
  return std::visit(
      [](const auto& state) -> FsmState {
        using T = std::decay_t<decltype(state)>;
        if constexpr (std::is_same_v<T, SitDown<PidType, LoggerType, RampFilterType, MovingAverageFilterType>>)
          return FsmState::SIT_DOWN;
        else if constexpr (std::is_same_v<T, StandUp<PidType, LoggerType, RampFilterType, MovingAverageFilterType>>)
          return FsmState::STAND_UP;
        else if constexpr (std::is_same_v<T, Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>>)
          return FsmState::NORMAL;
        else if constexpr (std::is_same_v<T, Recover<PidType, LoggerType, RampFilterType, MovingAverageFilterType>>)
          return FsmState::RECOVERY;
        else if constexpr (std::is_same_v<T, Upstairs<PidType, LoggerType, RampFilterType, MovingAverageFilterType>>)
          return FsmState::UPSTAIRS;
      },
      FSM_variant_);
}

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
bool BipedalWheelCore<PidType, LoggerType, RampFilterType, MovingAverageFilterType>::solveMpc(
    LegIndex side, double leg_length, const Eigen::Matrix<double, STATE_DIM, 1>& state,
    const Eigen::Matrix<double, STATE_DIM, 1>& reference,
    const Eigen::Matrix<double, CONTROL_DIM, 1>& input_min,
    const Eigen::Matrix<double, CONTROL_DIM, 1>& input_max, double vel_min, double vel_max,
    Eigen::Matrix<double, CONTROL_DIM, 1>& control)
{
  auto& solver = (side == bipedal_wheel_core::LEFT) ? mpc_left_solver_ : mpc_right_solver_;
  if (solver == nullptr || !mpc_lpv_1d_.initialized())
    return false;

  ScheduledMpcCache scheduled_cache;
  if (!mpc_lpv_1d_.evaluate(leg_length, scheduled_cache) || !solver->updateScheduledCache(scheduled_cache) ||
      !solver->updateInputBounds(input_min, input_max) || !solver->updateVelStateBounds(vel_min, vel_max))
  {
    return false;
  }
  return solver->solve(state, reference, control);
}

}  // namespace bipedal_wheel_core
