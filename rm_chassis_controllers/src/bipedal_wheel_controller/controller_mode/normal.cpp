//
// Created by guanlin on 25-9-3.
//

#include "bipedal_wheel_controller/controller_mode/normal.h"
#include "bipedal_wheel_controller/controller.h"
#include "bipedal_wheel_controller/helper_functions.h"
#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/controller_interface.h"
#include "bipedal_wheel_controller/definitions.h"
#include "hardware_interface/joint_command_interface.h"
#include "rm_common/filters/filters.h"
#include "rm_msgs/ChassisCmd.h"

#include <ros/time.h>
#include <ros/duration.h>
#include <control_toolbox/pid.h>
#include <ros/console.h>
#include <Eigen/Dense>

#include <cmath>
#include <vector>
#include <memory>

namespace rm_chassis_controllers
{
Normal::Normal(BipedalControllerInterface* controller_,
               const std::vector<hardware_interface::JointHandle*>& joint_handles,
               const std::vector<control_toolbox::Pid*>& pid_legs, control_toolbox::Pid* pid_yaw_vel,
               control_toolbox::Pid* pid_theta_diff, control_toolbox::Pid* pid_roll,
               control_toolbox::Pid* pid_wheel_vel_diff)
  : ModeBase(controller_)
  , joint_handles_(joint_handles)
  , pid_legs_(pid_legs)
  , pid_yaw_vel_(pid_yaw_vel)
  , pid_theta_diff_(pid_theta_diff)
  , pid_roll_(pid_roll)
  , pid_wheel_vel_diff_(pid_wheel_vel_diff)
{
  leftSupportForceAveragePtr_ = std::make_shared<MovingAverageFilter<double>>(4);
  rightSupportForceAveragePtr_ = std::make_shared<MovingAverageFilter<double>>(4);
  if (controller_->getLegThresholdParams() != nullptr)
    unstick_threshold = controller_->getLegThresholdParams()->unstick_threshold;
}

void Normal::execute(const ros::Time& time, const ros::Duration& period)
{
  const auto& bias_params = controller->getBiasParams();
  if (!controller->getStateChange())
  {
    ROS_INFO("[balance] Enter NORMAL");
    controller->clearStatus();
    jump_phase_ = JumpPhase::IDLE;
    pos_des_ = 0.0f;
    controller->setStateChange(true);
  }

  // Get lastest states
  const auto& chassis_state = controller->getChassisState();
  auto& left_leg_state = controller->getLegState(LEFT);
  auto& right_leg_state = controller->getLegState(RIGHT);
  const auto& left_pos = left_leg_state.vmc->getPos();
  const auto& right_pos = right_leg_state.vmc->getPos();
  const auto& left_spd = left_leg_state.vmc->getSpd();
  const auto& right_spd = right_leg_state.vmc->getSpd();

  // Stand up decation
  if (abs(left_leg_state.x[CHASSIS_PITCH]) < 0.2 && (abs(left_leg_state.x[LEG_THETA] + right_leg_state.x[LEG_THETA]) / 2.0f) < 0.2)
  {
    protect_flag_ = false;
    if (!controller->getCompleteStand())
    {
      controller->setCompleteStand(true);
    }
  }

  auto vel_cmd = controller->getVelCmd();
  double current_leg_length = (left_pos.L0 + right_pos.L0) / 2.0f;
  static double last_vel_cmd_x = vel_cmd.x;
  vel_direction_ = (vel_cmd.x - last_vel_cmd_x) > 0 ?
                       VEL_DIRECTION::POSITIVE :
                       (vel_cmd.x - last_vel_cmd_x) < 0 ? VEL_DIRECTION::NEGATIVE : vel_direction_;
  last_vel_cmd_x = vel_cmd.x;
  if (abs(chassis_state.x_vel) < 0.1f && abs(vel_cmd.x) < 0.01f)
  {
    controller->setMoveFlag(false);
    if (pos_offset_flag_)
    {
      pos_offset_flag_ = false;
      pos_des_ = current_leg_length * sin(-(left_leg_state.x(LEG_THETA) + right_leg_state.x(LEG_THETA)) / 2.0f) +
                 vel_direction_ * bias_params->x;
    }
  }

  double friction_circle = chassis_state.x_vel * chassis_state.angular_vel.z;
  double friction_circle_alpha = abs(friction_circle) > 10.0f ? (10.0f / abs(friction_circle)) : 1.0f;
  // PID
  double yaw_torque = pid_yaw_vel_->computeCommand(friction_circle_alpha * vel_cmd.z - chassis_state.angular_vel.z, period);
  double theta_diff = right_pos.theta - left_pos.theta;
  double theta_diff_torque = pid_theta_diff_->computeCommand(theta_diff, period);
  double roll_force = pid_roll_->computeCommand(0. - chassis_state.roll, period);
  // LQR
  Matrix<double, 4, 12> lqr_gain_coefficients = controller->getCoeffs();
  Matrix<double, 2, 6> left_lqr_gain, right_lqr_gain;
  left_lqr_gain.setZero();
  right_lqr_gain.setZero();

  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      left_lqr_gain(i, j) = lqr_gain_coefficients(0, i + 2 * j) * pow(left_pos.L0, 3) + lqr_gain_coefficients(1, i + 2 * j) * pow(left_pos.L0, 2) +
                     lqr_gain_coefficients(2, i + 2 * j) * left_pos.L0 + lqr_gain_coefficients(3, i + 2 * j);
      right_lqr_gain(i, j) = lqr_gain_coefficients(0, i + 2 * j) * pow(right_pos.L0, 3) + lqr_gain_coefficients(1, i + 2 * j) * pow(right_pos.L0, 2) +
                      lqr_gain_coefficients(2, i + 2 * j) * right_pos.L0 + lqr_gain_coefficients(3, i + 2 * j);
    }
  }

  Eigen::Matrix<double, CONTROL_DIM, 1> left_control_input, right_control_input;
  left_control_input.setZero();
  right_control_input.setZero();
  auto left_state = left_leg_state.x;
  auto right_state = right_leg_state.x;
  Matrix<double, 6, 1> left_reference_state, right_reference_state;

  left_reference_state.setZero();
  right_reference_state.setZero();

  // apply different bias
  if (controller->getCompleteStand())
  {
    if (controller->getBaseState() != rm_msgs::ChassisCmd::RAW)
    {
      left_reference_state(CHASSIS_VEL) = right_reference_state(CHASSIS_VEL) = friction_circle_alpha * vel_cmd.x;
      double theta_bias = bias_params->theta;
      // left_state(THETA) -= theta_bias;
      // right_state(THETA) -= theta_bias;

      // TODO make sure we should use += or = 
      left_reference_state(LEG_THETA) += theta_bias;
      right_reference_state(LEG_THETA) += theta_bias;
      if (!controller->getMoveFlag())
      {
        pos_offset_flag_ = true;
      }
      else
      {
        pos_des_ = bias_params->x;
      }
    }
    else
    {
      // raw move but bug
      left_reference_state(CHASSIS_POS) = right_reference_state(CHASSIS_POS) = 0.0f;
      left_reference_state(CHASSIS_VEL) = right_reference_state(CHASSIS_VEL) = vel_cmd.x;
      //      left_reference_state(VEL) = right_reference_state(VEL) = 0.0f;
      left_reference_state(LEG_THETA) = right_reference_state(LEG_THETA) = bias_params->raw_theta;
      left_reference_state(CHASSIS_PITCH) = right_reference_state(CHASSIS_PITCH) = bias_params->raw_pitch;
      // left_state(THETA) -= bias_params_->raw_theta;
      // right_state(THETA) -= bias_params_->raw_theta;
      // left_state(PITCH) -= bias_params_->raw_pitch;
      // right_state(PITCH) -= bias_params_->raw_pitch;
    }
    left_reference_state(CHASSIS_POS) = right_reference_state(CHASSIS_POS) = pos_des_;
    if (protect_flag_)
    {
      left_reference_state(CHASSIS_VEL) = right_reference_state(CHASSIS_VEL) = 0.0f;
      desired_leg_length = controller->getDefaultLegLength();
    }
    else
    {
      desired_leg_length = controller->getLegCmd();
    }
  }
  else
  {
    desired_leg_length = controller->getDefaultLegLength();
  }

  left_state -= left_reference_state;
  right_state -= right_reference_state;

  clamp(left_state(CHASSIS_VEL), -1.1f, 1.1f);
  clamp(right_state(CHASSIS_VEL), -1.1f, 1.1f);

  const double k_pitch = -0.1f, b = 0.35f;
  double pitch_error_clamp = k_pitch * chassis_state.x_vel + b;
  clamp(left_state(CHASSIS_PITCH), -pitch_error_clamp, pitch_error_clamp);
  clamp(right_state(CHASSIS_PITCH), -pitch_error_clamp, pitch_error_clamp);
  clamp(left_state(LEG_THETA), -0.6f, 0.6f);
  clamp(right_state(LEG_THETA), -0.6f, 0.6f);

  left_control_input = left_lqr_gain * (-left_state);
  right_control_input = right_lqr_gain * (-right_state);

  // Compute leg thrust
  bool jump_phase_changed = false;
  auto model_params = controller->getModelParams();
  auto control_params = controller->getControlParams();
  double wheel_vel_diff = abs(joint_handles_[4]->getVelocity()) - abs(joint_handles_[5]->getVelocity());
  double gravity = model_params->f_gravity, left_spring_force = controller->f_spring_force(left_pos.L0),
         right_spring_force = controller->f_spring_force(right_pos.L0);
  double left_turn_compensation_force =
      model_params->M * friction_circle * left_pos.L0 / controller->getChassisGeometryParams()->wheel_track;
  double right_turn_compensation_force =
      model_params->M * friction_circle * right_pos.L0 / controller->getChassisGeometryParams()->wheel_track;
  double left_pid_force{}, right_pid_force{}, wheel_deff_torque{};
  Eigen::Matrix<double, 2, 1> leg_axial_force;
  leg_axial_force.setZero();
  // check jump
  if (jump_phase_ == JumpPhase::IDLE &&
      ros::Time::now() - lastJumpTime_ > ros::Duration(control_params->jumpOverTime_) && controller->getJumpCmd())
      // 检查是否大于设定的跳跃间隔时间
  {
    jump_phase_changed = true;
    jump_phase_ = JumpPhase::LEG_RETRACTION;
    ROS_INFO("[balance] Jump start");

  }
  if (jump_phase_ == JumpPhase::IDLE)
  {
    static double last_left_length_des = desired_leg_length, last_right_length_des = desired_leg_length;
    double left_length_des = controller->getCompleteStand() ? (0.8 * desired_leg_length + 0.2 * last_left_length_des) :
                                                              controller->getDefaultLegLength();
    double right_length_des = controller->getCompleteStand() ? (0.8 * desired_leg_length + 0.2 * last_right_length_des) :
                                                               controller->getDefaultLegLength();
    last_left_length_des = left_length_des;
    last_right_length_des = right_length_des;
    left_pid_force = pid_legs_[LEFT]->computeCommand(left_length_des - current_leg_length, period);
    right_pid_force = pid_legs_[RIGHT]->computeCommand(right_length_des - current_leg_length, period);
    left_pid_force = abs(left_pid_force) > 150 ? std::copysign(1, left_pid_force) * 150 : left_pid_force;
    right_pid_force = abs(right_pid_force) > 150 ? std::copysign(1, right_pid_force) * 150 : right_pid_force;
    leg_axial_force[LEFT] = left_pid_force - left_turn_compensation_force + gravity / cos(left_pos.theta) + roll_force - left_spring_force;
    leg_axial_force[RIGHT] = right_pid_force + right_turn_compensation_force + gravity / cos(right_pos.theta) - roll_force - right_spring_force;
    wheel_deff_torque = controller->getBaseState() == rm_msgs::ChassisCmd::RAW ?
                       std::copysign(1, vel_cmd.z) * (pid_wheel_vel_diff_->computeCommand(wheel_vel_diff, period)) :
                       0.0f;
  }
  else
  {
    desired_leg_length = JUMP_PHASE_LEG_LENGTHS[jump_phase_].second;
    double s_left = (left_pos.L0 - 0.12) / (0.35 - 0.11);
    double s_right = (right_pos.L0 - 0.12) / (0.35 - 0.11);
    switch (jump_phase_)
    {
      case JumpPhase::LEG_RETRACTION:
      {
        if (jump_phase_changed)
        {
          ROS_INFO("[balance] ENTER LEG_RETRACTION");
          jump_phase_changed = false;
        }
        leg_axial_force(LEFT) = pid_legs_[LEFT]->computeCommand((desired_leg_length - 0.02f) - current_leg_length, period) +
                      gravity / cos(left_pos.theta) + roll_force - left_spring_force;
        leg_axial_force(RIGHT) = pid_legs_[RIGHT]->computeCommand((desired_leg_length - 0.02f) - current_leg_length, period) +
                       gravity / cos(right_pos.theta) - roll_force - right_spring_force;
        if (current_leg_length < desired_leg_length + 0.02f)
        {
          stage_duration_time++;
        }
        if (stage_duration_time >= 10)
        {
          stage_duration_time = 0;
          jump_phase_changed = true;
          jump_phase_ = JumpPhase::JUMP_UP;
        }
        break;
      }
      case JumpPhase::JUMP_UP:
        leg_axial_force(LEFT) = control_params->jump_up_force *
                            (1 - 3 * pow(s_left, 2) + 
                            2 * pow(s_left, 3)) + gravity;
        leg_axial_force(RIGHT) = control_params->jump_up_force *
                            (1 - 3 * pow(s_right, 2) + 
                            2 * pow(s_right, 3)) + gravity;
        if (jump_phase_changed)
        {
          ROS_INFO("[balance] ENTER JUMP_UP");
          jump_phase_changed = false;
        }
        if (current_leg_length > desired_leg_length - 0.01f)
        {
          stage_duration_time++;
        }
        if (stage_duration_time >= 4)
        {
          stage_duration_time = 0;
          jump_phase_changed = true;
          jump_phase_ = JumpPhase::OFF_GROUND;
          off_ground_start_time_ = time;
        }
        break;
      case JumpPhase::OFF_GROUND:
        if (jump_phase_changed)
        {
          ROS_INFO("[balance] ENTER OFF_GROUND");
          jump_phase_changed = false;
        }
        if (time - off_ground_start_time_ < ros::Duration(0.2))
        {
          const double s_left_flip = 1 - s_left;
          const double s_right_flip = 1 - s_right;
          leg_axial_force(LEFT) = -control_params->off_ground_force * (1 - 3 * pow(s_left_flip, 2) + 2 * pow(s_left_flip, 3)) -
                        left_spring_force;
          leg_axial_force(RIGHT) = -control_params->off_ground_force * (1 - 3 * pow(s_right_flip, 2) + 2 * pow(s_right_flip, 3)) -
                         right_spring_force;
        }
        else
        {
          if (!jump_phase_changed) {
            ROS_INFO("[balance] landing");
            jump_phase_changed = true;
          }
          desired_leg_length = 0.21;
        }
        break;
    }
  }

  // Unstick detection
  Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> left_lqr_gain_unstick{}, right_lqr_gain_unstick{};
  left_lqr_gain_unstick.setZero();
  right_lqr_gain_unstick.setZero();
  left_lqr_gain_unstick.block<1, 2>(1, 0) = left_lqr_gain.block<1, 2>(1, 0);
  right_lqr_gain_unstick.block<1, 2>(1, 0) = right_lqr_gain.block<1, 2>(1, 0);

  static bool last_left_unstick{ false }, last_right_unstick{ false };
  bool left_unstick{ false }, right_unstick{ false };
  if (jump_phase_ == JumpPhase::OFF_GROUND)
  {
    left_unstick = right_unstick = true;
  }
  else if (controller->getCompleteStand() && jump_phase_ != JumpPhase::LEG_RETRACTION &&
           controller->getBaseState() == rm_msgs::ChassisCmd::FOLLOW)
  {
    left_unstick =
        unstickDetection(last_left_unstick ? left_pid_force : left_leg_state.vmc->getForceReal().F + left_spring_force,
                         left_control_input(LEG_TORQUE), left_spd.dL0, left_pos.L0, chassis_state.linear_acc.z, model_params,
                         left_leg_state.x, leftSupportForceAveragePtr_, period);
    right_unstick =
        unstickDetection(last_right_unstick ? right_pid_force : right_leg_state.vmc->getForceReal().F + right_spring_force,
                         right_control_input(LEG_TORQUE), right_spd.dL0, right_pos.L0, chassis_state.linear_acc.z, model_params,
                         right_leg_state.x, rightSupportForceAveragePtr_, period);
  }
  bool unstick[2]{};
  unstick[0] = left_unstick;
  unstick[1] = right_unstick;
  last_left_unstick = left_unstick;
  last_right_unstick = right_unstick;
  Matrix<double, 2, 1> leg_support_force{};
  leg_support_force(LEFT) = leftSupportForceAveragePtr_->output();
  leg_support_force(RIGHT) = rightSupportForceAveragePtr_->output();
  controller->pubLQRStatus(-left_state, -right_state, left_reference_state, right_reference_state, left_control_input, right_control_input, leg_support_force, unstick);

  updateUnstick(left_unstick, right_unstick);
  //  left_unstick = right_unstick = false;
  bool unstick_flag = left_unstick && right_unstick;
  if ((controller->getCompleteStand() && unstick_flag && jump_phase_ != JumpPhase::LEG_RETRACTION) ||
      jump_phase_ == JumpPhase::OFF_GROUND)
  {
    left_control_input = left_lqr_gain_unstick * (-left_state);
    right_control_input = right_lqr_gain_unstick * (-right_state);
  }

  // Control
  double left_motor_torque[2], right_motor_torque[2];
  
  left_leg_state.vmc->leg_conv(leg_axial_force[LEFT], left_control_input(LEG_TORQUE) + theta_diff_torque, left_motor_torque);
  right_leg_state.vmc->leg_conv(leg_axial_force[RIGHT], right_control_input(LEG_TORQUE) - theta_diff_torque, right_motor_torque);
  
  double left_wheel_cmd = unstick_flag ? 0. : left_control_input(WHEEL_TORQUE) - yaw_torque - wheel_deff_torque;
  double right_wheel_cmd = unstick_flag ? 0. : right_control_input(WHEEL_TORQUE) + yaw_torque - wheel_deff_torque;
  LegCommand left_cmd = { leg_axial_force[LEFT], left_control_input(LEG_TORQUE) + theta_diff_torque, { left_motor_torque[0], left_motor_torque[1] } },
             right_cmd = { leg_axial_force[RIGHT], right_control_input(LEG_TORQUE) - theta_diff_torque, { right_motor_torque[0], right_motor_torque[1] } };

  // upstairs
  if (jump_phase_ == JumpPhase::IDLE && controller->getCompleteStand() && abs(left_state(0) + right_state(0)) / 2.0f > 0.50 &&
      abs(vel_cmd.x) > 0.1 && abs(left_state(3)) > 0.1 && ((left_pos.L0 + right_pos.L0) / 2.0f) > 0.30 &&
      desired_leg_length > 0.30)
  {
    controller->setMode(BalanceMode::UPSTAIRS);
    controller->setStateChange(false);
    controller->setJumpCmd(false);
    controller->setCompleteStand(false);
    left_wheel_cmd = right_wheel_cmd = 0;
    ROS_INFO("[balance] Exit NORMAL");
  }

  if (desired_leg_length < 0.22)
  {
    // Protection
    if ((abs(left_state(0)) > 0.6f || abs(right_state(0)) > 0.6f || abs(chassis_state.pitch) > 0.4 ||
         abs(chassis_state.roll) > 0.4) ||
        (abs(left_control_input(0)) + abs(right_control_input(0)) / 2.0f > 30.0f) ||
        (abs(chassis_state.x_vel - left_reference_state(CHASSIS_VEL)) > 5.0f && abs(chassis_state.x_vel - vel_cmd.x) > 5.0f))
    {
      protect_flag_ = true;
      desired_leg_length = controller->getDefaultLegLength();
      left_leg_state.x(CHASSIS_POS) = right_leg_state.x(CHASSIS_POS) = 0;
      controller->setMode(BalanceMode::PROTECT);
      controller->setStateChange(false);
      controller->setCompleteStand(false);
      controller->setJumpCmd(false);
      setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });
      ROS_INFO("[balance] Exit NORMAL");
    }
  }

  double enter_sitdown_theta_threshold =
      controller->getDown5cmStairFlag() ? control_params->down5cmStairThetaThreshold : 1.0;
  double enter_sitdown_pitch_threshold =
      controller->getDown5cmStairFlag() ? control_params->down5cmStairPitchThreshold : 0.6;
  // Protection to sit_down
  if (abs(left_state(LEG_THETA)) > enter_sitdown_theta_threshold || abs(right_state(LEG_THETA)) > enter_sitdown_theta_threshold ||
      abs(chassis_state.pitch) > enter_sitdown_pitch_threshold || abs(chassis_state.roll) > 0.8 ||
      controller->getOverturn() || abs(theta_diff) > 1.0 || controller->getBaseState() == rm_msgs::ChassisCmd::FALLEN)
  {
    left_leg_state.x(CHASSIS_POS) = right_leg_state.x(CHASSIS_POS) = 0;
    controller->setMode(BalanceMode::SIT_DOWN);
    controller->setStateChange(false);
    controller->setCompleteStand(false);
    controller->setJumpCmd(false);
    controller->setDown5cmStairFlag(false);
    setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });
    ROS_INFO("[balance] Exit NORMAL");
  }
  setJointCommands(joint_handles_, left_cmd, right_cmd, left_wheel_cmd, right_wheel_cmd);
}

// TODO replace state with specific param in state
double Normal::calculateSupportForce(double leg_axial_force, double leg_angle_torque, double leg_length, 
                                     const double& leg_length_dot, double acc_z,
                                     const Eigen::Matrix<double, STATE_DIM, 1>& state,
                                     const std::shared_ptr<ModelParams>& model_params, const ros::Duration& period)
{
  static double last_chassis_height_ddot = acc_z - model_params->g, 
                last_leg_theta_dot = state(1),
                last_leg_theta_ddot = (state(1) - last_leg_theta_dot) / period.toSec(), 
                last_ddot_leg_len, 
                leg_length_dot_history[3]{};
  leg_length_dot_history[0] = leg_length_dot_history[1];
  leg_length_dot_history[1] = leg_length_dot_history[2];
  leg_length_dot_history[2] = leg_length_dot;

  double leg_vertical_force = leg_axial_force * cos(state(0)) + leg_angle_torque * sin(state(0)) / leg_length;
  // lp filter
  double chassis_height_ddot = 0.3 * (acc_z - model_params->g) + 0.7 * last_chassis_height_ddot;
  double leg_theta_ddot = 0.3 * ((state(1) - last_leg_theta_dot) / period.toSec()) + 0.7 * last_leg_theta_ddot;
  double leg_length_ddot = 0.3 * (leg_length_dot_history[2] - leg_length_dot_history[0]) / 2 * period.toSec() + 0.7 * last_ddot_leg_len;
  last_leg_theta_dot = state(1);
  last_leg_theta_ddot = leg_theta_ddot;
  last_ddot_leg_len = leg_length_ddot;
  double wheel_height_ddot = chassis_height_ddot - leg_length_ddot * cos(state(0)) + 2 * leg_length_dot * state(1) * sin(state(0)) +
                   +leg_length * (leg_theta_ddot * sin(state(0)) + leg_length * state(1) * state(1) * cos(state(0)));
  double support_force = model_params->m_w * wheel_height_ddot + model_params->m_w * model_params->g + leg_vertical_force;

  return support_force;
}


// TODO apply last unstick flag
bool Normal::unstickDetection(const double& leg_axial_force, const double& leg_angle_torque, const double& leg_length_dot,
                              const double& leg_length, const double& acc_z,
                              const std::shared_ptr<ModelParams>& model_params, 
                              const Eigen::Matrix<double, STATE_DIM, 1>& state,
                              const std::shared_ptr<MovingAverageFilter<double>>& supportForceAveragePtr,
                              const ros::Duration& period)
{
  static bool maybe_change = false, last_unstick_flag = false;
  static ros::Time judge_time;
  double support_force = calculateSupportForce(leg_axial_force, leg_angle_torque, leg_length, leg_length_dot, acc_z, state, model_params, period);
  supportForceAveragePtr->input(support_force);
  bool unstick_flag = supportForceAveragePtr->output() < unstick_threshold;

  if (unstick_flag != last_unstick_flag)
  {
    if (!maybe_change)
    {
      judge_time = ros::Time::now();
      maybe_change = true;
    }
    else
    {
      if (ros::Time::now() - judge_time > ros::Duration(0.1))
      {
        last_unstick_flag = unstick_flag;
      }
    }
  }
  else
  {
    maybe_change = false;
  }
  return unstick_flag;
}
}  // namespace rm_chassis_controllers
