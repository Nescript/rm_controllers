//
// Created by guanlin on 25-8-28.
//

#include "bipedal_wheel_controller/controller.h"

#include <angles/angles.h>
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pluginlib/class_list_macros.hpp>
#include <unsupported/Eigen/MatrixFunctions>

namespace rm_chassis_controllers
{
BipedalController::~BipedalController()
{
  power_status_sub_.shutdown();
  power_fit_logger_.reset();
}

bool BipedalController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");
  //  gimbal_imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("gimbal_imu");
  const std::pair<const char*, hardware_interface::JointHandle*> table[] = {
    { "left_hip_joint", &left_hip_joint_handle_ },     { "left_knee_joint", &left_knee_joint_handle_ },
    { "right_hip_joint", &right_hip_joint_handle_ },   { "right_knee_joint", &right_knee_joint_handle_ },
    { "left_wheel_joint", &left_wheel_joint_handle_ }, { "right_wheel_joint", &right_wheel_joint_handle_ }
  };
  auto* joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  for (const auto& t : table)
  {
    *t.second = joint_interface->getHandle(t.first);
    joint_handles_.push_back(t.second);
  }

  if (!setupParams(controller_nh))
  {
    ROS_ERROR("[balance] Failed to setup parameters");
    return false;
  }

  mode_manager_ = std::make_shared<ModeManager>(this, controller_nh, joint_handles_);

  d_srv_ =
      new dynamic_reconfigure::Server<rm_chassis_controllers::LQRWeightConfig>(ros::NodeHandle(controller_nh, "lqr"));
  dynamic_reconfigure::Server<rm_chassis_controllers::LQRWeightConfig>::CallbackType cb =
      boost::bind(&BipedalController::reconfigCB, this, _1, _2);
  d_srv_->setCallback(cb);

  auto legCmdCallback = [this](const rm_msgs::LegCmd::ConstPtr& msg) {
    legCmd_ = msg->leg_length;
    jumpCmd_ = msg->jump;
  };
  auto recoveryLegSpdTurnbackCb = [this](const std_msgs::Bool::ConstPtr& msg) { setRecoveryLegSpdTurnback(msg->data); };
  leg_cmd_sub_ = controller_nh.subscribe<rm_msgs::LegCmd>("/leg_cmd", 5, legCmdCallback);
  recovery_leg_spd_turnback_sub_ =
      controller_nh.subscribe<std_msgs::Bool>("/recovery_leg_spd_turnback", 1, recoveryLegSpdTurnbackCb);
  unstick_pub_ = controller_nh.advertise<std_msgs::Bool>("unstick", 1);
  upstair_status_pub_ = controller_nh.advertise<rm_msgs::LeggedUpstairStatus>("upstair_status", 1);

  legged_chassis_status_pub_.reset(
      (new realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisStatus>(controller_nh, "legged_chassis_status", 1)));
  legged_chassis_mode_pub_.reset(
      (new realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisMode>(controller_nh, "legged_chassis_mode", 1)));
  lqr_status_pub_.reset(
      (new realtime_tools::RealtimePublisher<rm_msgs::LeggedLQRStatus>(controller_nh, "lqr_status", 1)));
  leg_state_[LEFT].x.setZero();
  leg_state_[RIGHT].x.setZero();

  // Slippage detection
  A_ << 1, 0.001f, 0, 1;
  H_ << 1, 0, 0, 1;
  Q_ << 1, 0, 0, 1;
  R_ << 200, 0, 0, 200;
  //  Q_ << 25, 0, 0, 2000;
  //  R_ << 800, 0, 0, 0.01;
  R_wheel_ = R_(0, 0);
  slip_R_wheel_ = slip_alpha_ * R_wheel_;
  B_.setZero();
  X_.setZero();
  U_.setZero();
  kalmanFilterPtr_ = std::make_shared<KalmanFilter<double>>(A_, B_, H_, Q_, R_);
  kalmanFilterPtr_->clear(X_);

  down_5cm_stair_srv_ =
      controller_nh.advertiseService("/down_5cm_stair", &BipedalController::down5cmStairSrvCallback, this);

  // Power model online identification
  double power_lambda = 0.999, power_alpha_ridge = 0.001;
  controller_nh.param("power_model/lambda", power_lambda, power_lambda);
  controller_nh.param("power_model/alpha_ridge", power_alpha_ridge, power_alpha_ridge);
  controller_nh.param("power_model/identification_enabled", power_identification_enabled_,
                      power_identification_enabled_);
  controller_nh.param("power_model/require_normal_state", power_require_normal_state_, power_require_normal_state_);
  controller_nh.param("power_model/reject_protected_samples", power_reject_protected_samples_,
                      power_reject_protected_samples_);
  controller_nh.param("power_model/filter_time_constant", power_filter_time_constant_, power_filter_time_constant_);
  controller_nh.param("power_model/measurement_delay", power_measurement_delay_, power_measurement_delay_);
  controller_nh.param("power_model/max_alignment_error", power_max_alignment_error_, power_max_alignment_error_);
  controller_nh.param("power_model/stationary_sample_period", power_stationary_sample_period_,
                      power_stationary_sample_period_);
  controller_nh.param("power_model/min_measurement", power_min_measurement_, power_min_measurement_);
  controller_nh.param("power_model/max_measurement", power_max_measurement_, power_max_measurement_);
  int minimum_samples = static_cast<int>(power_min_samples_);
  controller_nh.param("power_model/minimum_samples", minimum_samples, minimum_samples);
  power_min_samples_ = static_cast<uint64_t>(std::max(0, minimum_samples));
  controller_nh.param("power_model/min_normalized_eigenvalue", power_min_normalized_eigenvalue_,
                      power_min_normalized_eigenvalue_);
  controller_nh.param("power_model/min_r2", power_min_r2_, power_min_r2_);
  controller_nh.param("power_model/max_rmse", power_max_rmse_, power_max_rmse_);
  power_model_.init(power_lambda, power_alpha_ridge);
  Eigen::Matrix<double, 12, 1> init_c;
  init_c.setZero();
  power_coeffs_rt_buffer_.initRT(init_c);
  controller_nh.param("power_model/metrics_rate", power_metrics_rate_, 1.0);
  if (!std::isfinite(power_metrics_rate_) || power_metrics_rate_ <= 0.0 ||
      !std::isfinite(power_lambda) || power_lambda <= 0 || power_lambda > 1 ||
      !std::isfinite(power_alpha_ridge) || power_alpha_ridge < 0 || power_min_samples_ < 13 ||
      !std::isfinite(power_filter_time_constant_) || power_filter_time_constant_ < 0.0 ||
      !std::isfinite(power_measurement_delay_) || power_measurement_delay_ < 0.0 ||
      !std::isfinite(power_max_alignment_error_) || power_max_alignment_error_ <= 0.0 ||
      !std::isfinite(power_stationary_sample_period_) || power_stationary_sample_period_ <= 0.0 ||
      !std::isfinite(power_min_measurement_) || !std::isfinite(power_max_measurement_) ||
      power_min_measurement_ < 0.0 || power_max_measurement_ <= power_min_measurement_ ||
      !std::isfinite(power_min_normalized_eigenvalue_) || power_min_normalized_eigenvalue_ < 0.0 ||
      !std::isfinite(power_min_r2_) || !std::isfinite(power_max_rmse_) || power_max_rmse_ <= 0.0)
  {
    ROS_ERROR("[PowerID] Invalid online identification parameters");
    return false;
  }
  power_session_id_ = std::to_string(ros::WallTime::now().toNSec()) + "-" +
      boost::filesystem::unique_path("%%%%%%%%").string();
  bool log_enabled;
  controller_nh.param("power_model/log_enabled", log_enabled, true);
  const char* log_dir = std::getenv("ROS_LOG_DIR");
  const char* ros_home = std::getenv("ROS_HOME");
  const char* user_home = std::getenv("HOME");
  std::string directory = log_dir ? log_dir :
      (std::string(ros_home ? ros_home : (user_home ? user_home : "/tmp")) +
       (ros_home ? "/log" : "/.ros/log"));
  directory += "/power_model";
  controller_nh.param("power_model/log_directory", directory, directory);
  if (log_enabled)
    power_fit_logger_.reset(new PowerFitLogger(directory, power_session_id_));
  power_prediction_pub_ = root_nh.advertise<std_msgs::Float64>("/power_model/predicted_power", 10);
  power_fit_pub_ = root_nh.advertise<rm_msgs::PowerModelFit>("/power_model/fit_metrics", 1, true);
  power_model_pub_ = root_nh.advertise<std_msgs::Float64MultiArray>("/power_model/c", 1, true);
  std::vector<double> initial_coefficients;
  if (controller_nh.getParam("power_model/initial_coefficients", initial_coefficients))
  {
    if (initial_coefficients.size() != 12)
    {
      ROS_ERROR("[PowerID] initial_coefficients must contain exactly 12 values");
      return false;
    }
    for (int i = 0; i < init_c.rows(); ++i)
      init_c(i) = initial_coefficients[i];
    if (!power_model_.setC(init_c))
    {
      ROS_ERROR("[PowerID] initial_coefficients must be finite and non-negative");
      return false;
    }
    power_coeffs_rt_buffer_.writeFromNonRT(init_c);
    accepted_power_model_valid_ = true;
    publishPowerCoefficients();
  }
  power_status_sub_ = root_nh.subscribe<rm_msgs::PowerManagementSampleAndStatusData>(
      "/rm_referee/power_management/sample_and_status", 10, &BipedalController::onPowerMeas, this);

  debugPub_ = std::make_shared<DebugDataPublisher>(controller_nh, "debug_data");
  return true;
}

bool BipedalController::down5cmStairSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  (void)req;
  triggerDown5cmStairAction();
  res.message = "down_5cm_stair flag set";
  res.success = true;
  return true;
}

void BipedalController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  if (!balance_state_changed_)
    mode_manager_->switchMode(balance_mode_);
  if (getBaseState() == rm_msgs::ChassisCmd::FALLEN)
  {
    balance_mode_ = SIT_DOWN;
    mode_manager_->switchMode(SIT_DOWN);
  }
  else if (!overturn_ && getBaseState() == rm_msgs::ChassisCmd::RECOVERY)
  {
    overturn_ = true;
    balance_mode_ = RECOVER;
    mode_manager_->switchMode(RECOVER);
  }
  updateEstimation(time, period);
  if (power_identification_enabled_)
    updatePowerModel(time, period);
  mode_manager_->getModeImpl()->execute(time, period);
  pubState();
}

void BipedalController::clearStatus()
{
  leg_state_[LEFT].x(2) = leg_state_[RIGHT].x(2) = 0;
}

void BipedalController::updateEstimation(const ros::Time& time, const ros::Duration& period)
{
  geometry_msgs::Vector3 gyro, acc;
  gyro.x = imu_handle_.getAngularVelocity()[0];
  gyro.y = imu_handle_.getAngularVelocity()[1];
  gyro.z = imu_handle_.getAngularVelocity()[2];
  acc.x = imu_handle_.getLinearAcceleration()[0];
  acc.y = imu_handle_.getLinearAcceleration()[1];
  acc.z = imu_handle_.getLinearAcceleration()[2];
  tf2::Transform odom2imu, imu2base, odom2base;
  geometry_msgs::Vector3 angular_vel_base{}, linear_acc_base{};
  double roll{}, pitch{}, yaw{};
  try
  {
    tf2::doTransform(gyro, angular_vel_base,
                     robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), time));
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_handle_.lookupTransform(imu_handle_.getFrameId(), "base_link", time);
    tf2::fromMsg(tf_msg.transform, imu2base);
    tf2::Quaternion odom2imu_quaternion;
    tf2::Vector3 odom2imu_origin;
    odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                                 imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
    odom2imu_origin.setValue(0, 0, 0);
    odom2imu.setOrigin(odom2imu_origin);
    odom2imu.setRotation(odom2imu_quaternion);
    odom2base = odom2imu * imu2base;
    quatToRPY(toMsg(odom2base).rotation, roll, pitch, yaw);

    tf_msg.transform = tf2::toMsg(odom2imu.inverse());
    tf_msg.header.stamp = time;
    tf2::doTransform(acc, linear_acc_base, tf_msg);

    tf2::Vector3 z_body(0, 0, 1);
    tf2::Vector3 z_world = tf2::quatRotate(odom2base.getRotation(), z_body);
    overturn_ = (abs(pitch) > 0.65 || abs(roll) > 0.8) && z_world.z() < 0.0;

    chassis_state_.angular_vel = angular_vel_base;
    chassis_state_.linear_acc = linear_acc_base;
    chassis_state_.roll = roll;
    chassis_state_.pitch = pitch;
    chassis_state_.yaw = yaw;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_ONCE("%s", ex.what());
    setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });
    return;
  }

  // vmc
  double left_angle[2]{}, right_angle[2]{};

  //  double left_pos[2]{}, left_spd[2]{}, right_pos[2]{}, right_spd[2]{};
  // [0]:hip_vmc_joint [1]:knee_vmc_joint
  left_angle[0] = left_hip_joint_handle_.getPosition() + M_PI;
  left_angle[1] = left_knee_joint_handle_.getPosition();
  right_angle[0] = right_hip_joint_handle_.getPosition() + M_PI;
  right_angle[1] = right_knee_joint_handle_.getPosition();

  //  gazebo
  //  left_angle[0] = left_hip_joint_handle_.getPosition() + M_PI_2;
  //  left_angle[1] = left_knee_joint_handle_.getPosition() - M_PI_2;
  //  right_angle[0] = right_hip_joint_handle_.getPosition() + M_PI_2;
  //  right_angle[1] = right_knee_joint_handle_.getPosition() - M_PI_2;

  // left vmc calc
  leg_state_[LEFT].vmc->calc_jacobian(left_angle[0], left_angle[1]);
  leg_state_[LEFT].vmc->leg_pos(left_angle[0], left_angle[1]);
  leg_state_[LEFT].vmc->leg_spd(left_hip_joint_handle_.getVelocity(), left_knee_joint_handle_.getVelocity());
  leg_state_[LEFT].vmc->leg_conv_t(left_hip_joint_handle_.getEffort(), left_knee_joint_handle_.getEffort());

  // right vmc calc
  leg_state_[RIGHT].vmc->calc_jacobian(right_angle[0], right_angle[1]);
  leg_state_[RIGHT].vmc->leg_pos(right_angle[0], right_angle[1]);
  leg_state_[RIGHT].vmc->leg_spd(right_hip_joint_handle_.getVelocity(), right_knee_joint_handle_.getVelocity());
  leg_state_[RIGHT].vmc->leg_conv_t(right_hip_joint_handle_.getEffort(), right_knee_joint_handle_.getEffort());

  const LegPos& left_pos = leg_state_[LEFT].vmc->getPos();
  const LegSpd& left_spd = leg_state_[LEFT].vmc->getSpd();
  const LegPos& right_pos = leg_state_[RIGHT].vmc->getPos();
  const LegSpd& right_spd = leg_state_[RIGHT].vmc->getSpd();
  const LegForce& left_F_real = leg_state_[LEFT].vmc->getForceReal();
  const LegForce& right_F_real = leg_state_[RIGHT].vmc->getForceReal();

  // Slippage_detection
  leftWheelVel = (left_wheel_joint_handle_.getVelocity() + angular_vel_base.y + left_spd.dTheta) * wheel_radius_;
  rightWheelVel = (right_wheel_joint_handle_.getVelocity() + angular_vel_base.y + right_spd.dTheta) * wheel_radius_;
  leftWheelVelAbsolute = leftWheelVel + left_pos.L0 * left_spd.dTheta * cos(left_pos.theta + pitch) +
                         left_spd.dL0 * sin(left_pos.theta + pitch);
  rightWheelVelAbsolute = rightWheelVel + right_pos.L0 * right_spd.dTheta * cos(right_pos.theta + pitch) +
                          right_spd.dL0 * sin(right_pos.theta + pitch);

  double wheel_vel_aver = (leftWheelVelAbsolute + rightWheelVelAbsolute) / 2.;
  R_(0, 0) = slip_flag_ ? slip_R_wheel_ : R_wheel_;
  if (itor >= sample_times_)
  {  // oversampling
    static double last_linear_acc_base_x = linear_acc_base.x;
    itor = 0;
    X_(0) = wheel_vel_aver;
    X_(1) = 0.2 * last_linear_acc_base_x + 0.8 * linear_acc_base.x;
    last_linear_acc_base_x = X_(1);
    kalmanFilterPtr_->predict(U_);
    kalmanFilterPtr_->update(X_, R_);
  }
  else
  {
    kalmanFilterPtr_->predict(U_);
    itor++;
  }
  auto x_hat_vel = kalmanFilterPtr_->getState();
  slip_flag_ = abs(x_hat_vel(0) - wheel_vel_aver) > 3.0;

  // update state
  leg_state_[LEFT].x[3] = state_ != RAW ? x_hat_vel(0) : 0;
  //  leg_state_[LEFT].x[3] = state_ != RAW ? wheel_vel_aver : 0;
  if (state_ != RAW && abs(leg_state_[LEFT].x[3]) <= 0.5f && abs(vel_cmd_.x) <= 0.01f)
  {
    leg_state_[LEFT].x[2] += state_ != RAW ? leg_state_[LEFT].x[3] * period.toSec() : 0;
  }
  else
  {
    setMoveFlag(true);
    leg_state_[LEFT].x[2] = 0.0;
  }
  leg_state_[LEFT].x[0] = (left_pos.theta + pitch);
  leg_state_[LEFT].x[1] = left_spd.dTheta + angular_vel_base.y;
  leg_state_[LEFT].x[4] = -pitch;
  leg_state_[LEFT].x[5] = -angular_vel_base.y;
  leg_state_[RIGHT].x = leg_state_[LEFT].x;
  leg_state_[RIGHT].x[0] = (right_pos.theta + pitch);
  leg_state_[RIGHT].x[1] = right_spd.dTheta + angular_vel_base.y;

  chassis_state_.x_vel = x_hat_vel(0);

  if (legged_chassis_status_pub_->trylock())
  {
    legged_chassis_status_pub_->msg_.linear_acc_base.clear();
    legged_chassis_status_pub_->msg_.roll = roll;
    legged_chassis_status_pub_->msg_.pitch = leg_state_[LEFT].x[4];
    legged_chassis_status_pub_->msg_.d_pitch = leg_state_[LEFT].x[5];
    legged_chassis_status_pub_->msg_.yaw = yaw;
    legged_chassis_status_pub_->msg_.d_yaw = angular_vel_base.z;
    legged_chassis_status_pub_->msg_.left_leg_length = left_pos.L0;
    legged_chassis_status_pub_->msg_.right_leg_length = right_pos.L0;
    legged_chassis_status_pub_->msg_.x = leg_state_[LEFT].x[2];
    legged_chassis_status_pub_->msg_.x_dot = leg_state_[LEFT].x[3];
    legged_chassis_status_pub_->msg_.left_leg_theta = leg_state_[LEFT].x[0];
    legged_chassis_status_pub_->msg_.left_leg_theta_dot = leg_state_[LEFT].x[1];
    legged_chassis_status_pub_->msg_.right_leg_theta = leg_state_[RIGHT].x[0];
    legged_chassis_status_pub_->msg_.right_leg_theta_dot = leg_state_[RIGHT].x[1];
    legged_chassis_status_pub_->msg_.linear_acc_base.push_back(linear_acc_base.x);
    legged_chassis_status_pub_->msg_.linear_acc_base.push_back(linear_acc_base.y);
    legged_chassis_status_pub_->msg_.linear_acc_base.push_back(linear_acc_base.z);
    legged_chassis_status_pub_->unlockAndPublish();
  }

  if (legged_chassis_mode_pub_->trylock())
  {
    legged_chassis_mode_pub_->msg_.mode = balance_mode_;
    legged_chassis_mode_pub_->msg_.mode_name = mode_manager_->getModeImpl()->name();
    legged_chassis_mode_pub_->unlockAndPublish();
  }

  debugPub_->add("left_spring_force", f_spring_force(left_pos.L0));
  debugPub_->add("right_spring_force", f_spring_force(right_pos.L0));
  debugPub_->add("left_F_real", left_F_real.F);
  debugPub_->add("right_F_real", right_F_real.F);
  debugPub_->add("wheel_vel_aver", wheel_vel_aver);
  debugPub_->publish();
}

void BipedalController::updatePowerModel(const ros::Time& time, const ros::Duration& period)
{
  const double dt = period.toSec();
  if (!std::isfinite(dt) || dt <= 0.0)
    return;
  const double v = chassis_state_.x_vel;
  const double omega = chassis_state_.angular_vel.z;
  // a 直接用 IMU base 系加速度，比速度差分噪声低一个量级
  const double raw_acceleration = chassis_state_.linear_acc.x;
  // alpha 没有直读来源，对角速度差分
  const double raw_angular_acceleration =
      !last_power_update_time_.isZero() ? (omega - last_yaw_rate_) / dt : 0.0;

  last_yaw_rate_ = omega;
  last_power_update_time_ = time;

  if (!std::isfinite(v) || !std::isfinite(omega) || !std::isfinite(raw_acceleration) ||
      !std::isfinite(raw_angular_acceleration))
    return;
  const double filter_gain = power_filter_time_constant_ <= 0.0 ?
                                 1.0 :
                                 std::min(1.0, dt / (power_filter_time_constant_ + dt));
  if (!power_filter_initialized_)
  {
    filtered_power_acceleration_ = raw_acceleration;
    filtered_power_angular_acceleration_ = raw_angular_acceleration;
    power_filter_initialized_ = true;
  }
  else
  {
    filtered_power_acceleration_ += filter_gain * (raw_acceleration - filtered_power_acceleration_);
    filtered_power_angular_acceleration_ +=
        filter_gain * (raw_angular_acceleration - filtered_power_angular_acceleration_);
  }

  // 仅在实时线程中将运动状态写入环形缓冲，剥离矩阵求解与跨线程竞争
  PowerSample s;
  s.stamp = time;
  s.v = v;
  s.omega = omega;
  s.a = filtered_power_acceleration_;
  s.alpha = filtered_power_angular_acceleration_;
  s.eligible = !power_require_normal_state_ ||
      (balance_mode_ == BalanceMode::NORMAL && complete_stand_ && !overturn_ && !slip_flag_ && !jumpCmd_ &&
       state_ == rm_msgs::ChassisCmd::FOLLOW);
  const int head = power_buffer_head_.load(std::memory_order_relaxed);
  auto& slot = power_buffer_[head];
  const uint64_t sequence = slot.sequence.load(std::memory_order_relaxed);
  slot.sequence.store(sequence + 1, std::memory_order_release);
  slot.stamp_ns.store(s.stamp.toNSec(), std::memory_order_relaxed);
  slot.v.store(s.v, std::memory_order_relaxed);
  slot.omega.store(s.omega, std::memory_order_relaxed);
  slot.a.store(s.a, std::memory_order_relaxed);
  slot.alpha.store(s.alpha, std::memory_order_relaxed);
  slot.eligible.store(s.eligible, std::memory_order_relaxed);
  slot.sequence.store(sequence + 2, std::memory_order_release);
  power_buffer_head_.store((head + 1) % POWER_BUFFER_SIZE, std::memory_order_release);
}

bool BipedalController::readPowerSample(int index, PowerSample& sample) const
{
  const auto& slot = power_buffer_[index];
  for (int attempt = 0; attempt < 3; ++attempt)
  {
    const uint64_t before = slot.sequence.load(std::memory_order_acquire);
    if (before & 1U)
      continue;
    const uint64_t stamp_ns = slot.stamp_ns.load(std::memory_order_relaxed);
    sample.stamp.fromNSec(stamp_ns);
    sample.v = slot.v.load(std::memory_order_relaxed);
    sample.omega = slot.omega.load(std::memory_order_relaxed);
    sample.a = slot.a.load(std::memory_order_relaxed);
    sample.alpha = slot.alpha.load(std::memory_order_relaxed);
    sample.eligible = slot.eligible.load(std::memory_order_relaxed);
    const uint64_t after = slot.sequence.load(std::memory_order_acquire);
    if (before == after && !(after & 1U))
      return stamp_ns != 0;
  }
  return false;
}

void BipedalController::onPowerMeas(const rm_msgs::PowerManagementSampleAndStatusData::ConstPtr& msg)
{
  if (!std::isfinite(msg->chassis_power) || msg->stamp.isZero() || msg->chassis_power < power_min_measurement_ ||
      msg->chassis_power > power_max_measurement_ ||
      (power_reject_protected_samples_ && msg->power_management_protection_info != 0))
    return;
  if (!power_identification_enabled_)
  {
    publishPowerFit();
    return;
  }
  chassis_power_meas_.store(msg->chassis_power, std::memory_order_relaxed);
  chassis_power_stamp_.store(msg->stamp.toSec(), std::memory_order_relaxed);
  const ros::Time measurement_time = msg->stamp - ros::Duration(power_measurement_delay_);

  // 在环形缓冲中找与功率时间戳最近的状态样本，对齐后喂给辨识器
  const int head = power_buffer_head_.load(std::memory_order_acquire);
  double best_dt = std::numeric_limits<double>::max();
  PowerSample best_sample;
  bool found = false;

  for (int i = 0; i < POWER_BUFFER_SIZE; ++i)
  {
    PowerSample s;
    if (!readPowerSample((head - 1 - i + POWER_BUFFER_SIZE) % POWER_BUFFER_SIZE, s))
      break;  // 尚未写满
    const double d = std::abs((s.stamp - measurement_time).toSec());
    if (d < best_dt)
    {
      best_dt = d;
      best_sample = s;  // 值拷贝，杜绝裸指针在后续计算期间被 RT 线程覆写
      found = true;
    }
    else
      break;  // 时间戳单调，越往前差越大
  }

  // 仅接受与延迟补偿后的功率测量足够接近的控制样本。
  if (found && best_dt < power_max_alignment_error_ && best_sample.eligible)
  {
    // Predict before fitting this measurement; also publish in the stationary dead zone.
    const double predicted_power = power_model_.predict(best_sample.v, best_sample.omega,
                                                         best_sample.a, best_sample.alpha);
    if (std::isfinite(predicted_power))
    {
      std_msgs::Float64 prediction;
      prediction.data = predicted_power;
      power_prediction_pub_.publish(prediction);
    }
    // 运动样本正常更新；静止样本降频保留，用于约束静态功率 c0。
    const bool excited = power_model_.isExcited(best_sample.v, best_sample.omega, best_sample.a, best_sample.alpha);
    const bool stationary_sample_due = !excited &&
        (power_last_stationary_sample_.isZero() ||
         msg->stamp - power_last_stationary_sample_ >= ros::Duration(power_stationary_sample_period_));
    if (excited || stationary_sample_due)
    {
      if (!power_model_.updateD(0, best_sample.v, best_sample.omega, best_sample.a, best_sample.alpha,
                               msg->chassis_power))
      {
        publishPowerFit();
        return;
      }
      power_stats_stamp_ = msg->stamp;
      if (stationary_sample_due)
        power_last_stationary_sample_ = msg->stamp;

      if (!power_model_.hasSufficientExcitation(power_min_samples_, power_min_normalized_eigenvalue_))
      {
        publishPowerFit();
        return;
      }

      // 在非实时回调线程中执行求解，并只发布达到拟合质量门槛的模型。
      const bool previous_model_valid = power_model_.isModelValid();
      const auto previous_coefficients = power_model_.getC();
      power_last_solve_success_ = power_model_.solve(power_min_samples_);
      power_last_model_accepted_ = false;
      if (power_last_solve_success_)
      {
        power_last_solve_stamp_ = msg->stamp;
        const auto candidate_fit = power_model_.evaluate();
        if (candidate_fit.metrics_valid && candidate_fit.r2_valid && candidate_fit.r2 >= power_min_r2_ &&
            candidate_fit.rmse <= power_max_rmse_)
        {
          power_last_model_accepted_ = true;
          accepted_power_model_valid_ = true;
          ++power_model_version_;
          power_coeffs_rt_buffer_.writeFromNonRT(power_model_.getC());
          publishPowerCoefficients();
        }
        else if (previous_model_valid)
          power_model_.setC(previous_coefficients);
        else
          power_model_.invalidateModel();
      }
      else
      {
        ROS_WARN_THROTTLE(2.0, "[PowerID] Solve failed, keeping the previous stable solution");
      }
    }
  }
  publishPowerFit();
}

void BipedalController::publishPowerFit()
{
  const auto now = ros::WallTime::now();
  if (!power_last_publish_.isZero() && (now - power_last_publish_).toSec() < 1.0 / power_metrics_rate_)
    return;
  power_last_publish_ = now;
  const auto fit = power_model_.evaluate();
  rm_msgs::PowerModelFit snapshot;
  snapshot.header.stamp = power_stats_stamp_;
  snapshot.session_id = power_session_id_;
  snapshot.model_version = power_model_version_;
  snapshot.last_solve_stamp = power_last_solve_stamp_;
  snapshot.sample_count = fit.sample_count;
  snapshot.model_valid = fit.model_valid;
  snapshot.metrics_valid = fit.metrics_valid;
  snapshot.r2_valid = fit.r2_valid;
  snapshot.last_solve_success = power_last_solve_success_;
  snapshot.model_accepted = power_last_model_accepted_;
  snapshot.lambda = power_model_.getLambda();
  snapshot.alpha_ridge = power_model_.getAlphaRidge();
  snapshot.rmse = fit.rmse;
  snapshot.r2 = fit.r2;
  snapshot.bias = fit.bias;
  const auto& c = power_model_.getC();
  std::copy(c.data(), c.data() + 12, snapshot.c.begin());
  power_fit_pub_.publish(snapshot);
  if (accepted_power_model_valid_)
    publishPowerCoefficients();
  if (power_fit_logger_)
    power_fit_logger_->enqueue(snapshot);
}

void BipedalController::publishPowerCoefficients()
{
  if (!accepted_power_model_valid_)
    return;
  const auto& coefficients = *power_coeffs_rt_buffer_.readFromNonRT();
  std_msgs::Float64MultiArray message;
  message.data.assign(coefficients.data(), coefficients.data() + coefficients.rows());
  power_model_pub_.publish(message);
}

void BipedalController::pubState()
{
  std_msgs::Bool msg;
  msg.data = mode_manager_->getModeImpl()->getUnstick();
  unstick_pub_.publish(msg);
}

void BipedalController::stopping(const ros::Time& time)
{
  balance_mode_ = BalanceMode::SIT_DOWN;
  setStateChange(false);
  setJointCommands(joint_handles_, { 0, 0, { 0., 0. } }, { 0, 0, { 0., 0. } });

  ROS_INFO("[balance] Controller Stop");
}

bool BipedalController::setupParams(ros::NodeHandle& controller_nh)
{
  model_params_ = std::make_shared<ModelParams>();
  control_params_ = std::make_shared<ControlParams>();
  bias_params_ = std::make_shared<BiasParams>();
  leg_threshold_params_ = std::make_shared<LegStateThresholdParams>();
  spring_params_ = std::make_shared<SpringParams>();
  chassis_geometry_params_ = std::make_shared<ChassisGeometryParams>();

  if (!setupModelParams(controller_nh) || !setupLQR(controller_nh) || !setupBiasParams(controller_nh) ||
      !setupControlParams(controller_nh) || !setupThresholdParams(controller_nh) || !setupSpringParams(controller_nh) ||
      !setupChassisGeometryParams(controller_nh))
    return false;
  return true;
}

bool BipedalController::setupModelParams(ros::NodeHandle& controller_nh)
{
  const std::pair<const char*, double*> tbl[] = { { "m_w", &model_params_->m_w },
                                                  { "m_p", &model_params_->m_p },
                                                  { "M", &model_params_->M },
                                                  { "i_w", &model_params_->i_w },
                                                  { "i_m", &model_params_->i_m },
                                                  { "i_p", &model_params_->i_p },
                                                  { "l", &model_params_->l },
                                                  { "L_weight", &model_params_->L_weight },
                                                  { "Lm_weight", &model_params_->Lm_weight },
                                                  { "g", &model_params_->g },
                                                  { "wheel_radius", &model_params_->r },
                                                  { "gravity_force", &model_params_->f_gravity } };

  for (const auto& e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }

  double l1, l2;
  if (!controller_nh.getParam("l1", l1) || !controller_nh.getParam("l2", l2))
  {
    ROS_ERROR("Param %s or %s not given (namespace: %s)", "l1", "l2", controller_nh.getNamespace().c_str());
    return false;
  }
  leg_state_[LEFT].vmc = std::make_shared<VMC>(l1, l2);
  leg_state_[RIGHT].vmc = std::make_shared<VMC>(l1, l2);

  if (!controller_nh.getParam("default_leg_length", default_leg_length_))
  {
    ROS_ERROR("Param %s not given (namespace: %s)", "default_leg_length", controller_nh.getNamespace().c_str());
    return false;
  }

  return true;
}

bool BipedalController::setupChassisGeometryParams(ros::NodeHandle& controller_nh)
{
  const std::pair<const char*, double*> tbl[] = { { "wheel_track", &chassis_geometry_params_->wheel_track },
                                                  { "chassis_high", &chassis_geometry_params_->chassis_height } };

  for (const auto& e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }

  return true;
}

bool BipedalController::setupLQR(ros::NodeHandle& controller_nh)
{
  // Set up weight matrices
  auto loadWeightMatrix = [](ros::NodeHandle& nh, const char* key, int dim) -> Eigen::VectorXd {
    std::vector<double> v;
    if (!nh.getParam(key, v) || static_cast<int>(v.size()) != dim)
      return Eigen::VectorXd::Constant(dim, std::numeric_limits<double>::quiet_NaN());
    return Eigen::VectorXd::Map(v.data(), dim);
  };
  Eigen::VectorXd q_diag = loadWeightMatrix(controller_nh, "q", STATE_DIM);
  Eigen::VectorXd r_diag = loadWeightMatrix(controller_nh, "r", CONTROL_DIM);
  if (!q_diag.allFinite() || !r_diag.allFinite())
    return false;
  q_.setZero();
  r_.setZero();
  q_.diagonal() = q_diag;
  r_.diagonal() = r_diag;

  // Continuous model \dot{x} = A x + B u
  std::vector<double> lengths;
  std::vector<Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>> ks;
  for (int i = 10; i < 40; i++)
  {
    double length = i / 100.0f;
    lengths.push_back(length);
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> a{};
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b{};
    generateAB(model_params_, a, b, length);
    Lqr<double> lqr(a, b, q_, r_);
    if (!lqr.computeK())
    {
      ROS_ERROR("Failed to compute K of LQR.");
      return false;
    }
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k = lqr.getK();
    if (length == 0.2f)
    {
      std::cout << "A: " << std::endl << a << std::endl;
      std::cout << "B: " << std::endl << b << std::endl;
      std::cout << "len: 0.2m LQR k: " << std::endl << k << std::endl;
    }
    ks.push_back(k);
  }
  polyfit(ks, lengths, coeffs_);

  config_.Q_theta = q_.diagonal()(0);
  config_.Q_d_theta = q_.diagonal()(1);
  config_.Q_x = q_.diagonal()(2);
  config_.Q_dx = q_.diagonal()(3);
  config_.Q_phi = q_.diagonal()(4);
  config_.Q_d_phi = q_.diagonal()(5);
  config_.R_T = r_.diagonal()(0);
  config_.R_Tp = r_.diagonal()(1);
  config_rt_buffer_.initRT(config_);

  return true;
}

bool BipedalController::setupBiasParams(ros::NodeHandle& controller_nh)
{
  const std::pair<const char*, double*> tbl[] = { { "x_bias", &bias_params_->x },
                                                  { "theta_bias", &bias_params_->theta },
                                                  { "pitch_bias", &bias_params_->pitch },
                                                  { "roll_bias", &bias_params_->roll },
                                                  { "raw_pitch_bias", &bias_params_->raw_pitch },
                                                  { "raw_theta_bias", &bias_params_->raw_theta } };

  for (const auto& e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  return true;
}

// [will unused]
bool BipedalController::setupControlParams(ros::NodeHandle& controller_nh)
{
  const std::pair<const char*, double*> tbl[] = {
    { "jumpOverTime", &control_params_->jumpOverTime_ },
    { "down5cmStairPitchThreshold", &control_params_->down5cmStairPitchThreshold },
    { "down5cmStairThetaThreshold", &control_params_->down5cmStairThetaThreshold },
    { "jump_up_force", &control_params_->jump_up_force },
    { "off_ground_force", &control_params_->off_ground_force }
  };
  for (const auto& e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  return true;
}

bool BipedalController::setupThresholdParams(ros::NodeHandle& controller_nh)
{
  const std::pair<const char*, double*> tbl[] = {
    { "under_lower_threshold", &leg_threshold_params_->under_lower },
    { "under_upper_threshold", &leg_threshold_params_->under_upper },
    { "front_lower_threshold", &leg_threshold_params_->front_lower },
    { "front_upper_threshold", &leg_threshold_params_->front_upper },
    { "behind_lower_threshold", &leg_threshold_params_->behind_lower },
    { "behind_upper_threshold", &leg_threshold_params_->behind_upper },
    { "upstair_exit_theta_threshold", &leg_threshold_params_->upstair_exit_theta_threshold },
    { "upstair_exit_length_threshold", &leg_threshold_params_->upstair_exit_length_threshold },
    { "upstair_des_theta", &leg_threshold_params_->upstair_des_theta },
    { "upstair_des_length", &leg_threshold_params_->upstair_des_length },
    { "unstick_threshold", &leg_threshold_params_->unstick_threshold },
    { "arrive_time_threshold", &leg_threshold_params_->arrive_time_threshold }
  };
  for (const auto& e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  return true;
}

bool BipedalController::setupSpringParams(ros::NodeHandle& controller_nh)
{
  const std::pair<const char*, double*> tbl[] = {
    { "spring_s2", &spring_params_->s2 },
    { "spring_s3", &spring_params_->s3 },
    { "spring_alpha_s", &spring_params_->alpha_s },
    { "spring_force", &spring_params_->f_spring },
  };

  for (const auto& e : tbl)
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  return true;
}

void BipedalController::polyfit(const std::vector<Eigen::Matrix<double, 2, 6>>& Ks, const std::vector<double>& L0s,
                                Eigen::Matrix<double, 4, 12>& coeffs)
{
  int N = L0s.size();
  Eigen::MatrixXd A(N, 4), B(N, 12);
  for (int i = 0; i < N; ++i)
  {
    A.block(i, 0, 1, 4) << pow(L0s[i], 3), pow(L0s[i], 2), L0s[i], 1.0;
    Eigen::Map<const Eigen::Matrix<double, 12, 1>> flat(Ks[i].data());
    B.row(i) = flat.transpose();
  }
  coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * B);
}

geometry_msgs::Twist BipedalController::odometry()
{
  geometry_msgs::Twist twist;
  if (mode_manager_->getModeImpl() != nullptr)
  {
    twist.linear.x = chassis_state_.x_vel;
    twist.angular.z = chassis_state_.angular_vel.z;
  }
  else
  {
    twist.linear.x = 0;
    twist.angular.z = 0;
  }
  return twist;
}

void BipedalController::pubLQRStatus(Eigen::Matrix<double, STATE_DIM, 1> left_error,
                                     Eigen::Matrix<double, STATE_DIM, 1> right_error,
                                     Eigen::Matrix<double, STATE_DIM, 1> left_ref,
                                     Eigen::Matrix<double, STATE_DIM, 1> right_ref,
                                     Eigen::Matrix<double, CONTROL_DIM, 1> u_left,
                                     Eigen::Matrix<double, CONTROL_DIM, 1> u_right,
                                     Eigen::Matrix<double, CONTROL_DIM, 1> F_leg_, const bool unstick[2]) const
{
  if (lqr_status_pub_->trylock())
  {
    int temp{};
    lqr_status_pub_->msg_.left_leg_error.clear();
    lqr_status_pub_->msg_.right_leg_error.clear();
    lqr_status_pub_->msg_.left_leg_ref.clear();
    lqr_status_pub_->msg_.right_leg_ref.clear();
    for (temp = 0; temp < 6; ++temp)
    {
      lqr_status_pub_->msg_.left_leg_error.push_back(left_error(temp));
      lqr_status_pub_->msg_.right_leg_error.push_back(right_error(temp));
      lqr_status_pub_->msg_.left_leg_ref.push_back(left_ref(temp));
      lqr_status_pub_->msg_.right_leg_ref.push_back(right_ref(temp));
    }
    lqr_status_pub_->msg_.left_leg_u.clear();
    lqr_status_pub_->msg_.right_leg_u.clear();
    lqr_status_pub_->msg_.F_leg.clear();
    lqr_status_pub_->msg_.unstick.clear();
    for (temp = 0; temp < 2; ++temp)
    {
      lqr_status_pub_->msg_.left_leg_u.push_back(u_left(temp));
      lqr_status_pub_->msg_.right_leg_u.push_back(u_right(temp));
      lqr_status_pub_->msg_.F_leg.push_back(F_leg_[temp]);
      lqr_status_pub_->msg_.unstick.push_back(unstick[temp]);
    }
    lqr_status_pub_->unlockAndPublish();
  }
}

void BipedalController::pubLegLenStatus(const bool& upstair_flag)
{
  rm_msgs::LeggedUpstairStatus msg;
  msg.upstair_flag = upstair_flag;
  upstair_status_pub_.publish(msg);
}

void BipedalController::reconfigCB(rm_chassis_controllers::LQRWeightConfig& config, uint32_t /*level*/)
{
  ROS_INFO("[LQR] Dynamic params change");
  if (!dynamic_reconfig_initialized_)
  {
    LQRConfig init_config = *config_rt_buffer_.readFromNonRT();
    config.Q_theta = init_config.Q_theta;
    config.Q_d_theta = init_config.Q_d_theta;
    config.Q_x = init_config.Q_x;
    config.Q_dx = init_config.Q_dx;
    config.Q_phi = init_config.Q_phi;
    config.Q_d_phi = init_config.Q_d_phi;
    config.R_T = init_config.R_T;
    config.R_Tp = init_config.R_Tp;
    config.x_bias = bias_params_->x;
    config.theta_bias = bias_params_->theta;
    config.raw_theta_bias = bias_params_->raw_theta;
    dynamic_reconfig_initialized_ = true;
  }
  LQRConfig config_non_rt{ .Q_theta = config.Q_theta,
                           .Q_d_theta = config.Q_d_theta,
                           .Q_x = config.Q_x,
                           .Q_dx = config.Q_dx,
                           .Q_phi = config.Q_phi,
                           .Q_d_phi = config.Q_d_phi,
                           .R_T = config.R_T,
                           .R_Tp = config.R_Tp };
  config_rt_buffer_.writeFromNonRT(config_non_rt);

  q_.diagonal() << config.Q_theta, config.Q_d_theta, config.Q_x, config.Q_dx, config.Q_phi, config.Q_d_phi;
  r_.diagonal() << config.R_T, config.R_Tp;

  std::vector<double> lengths;
  std::vector<Eigen::Matrix<double, CONTROL_DIM, STATE_DIM>> ks;
  for (int i = 10; i < 40; i++)
  {
    double length = i / 100.;
    lengths.push_back(length);
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> a{};
    Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> b{};
    generateAB(model_params_, a, b, length);
    Lqr<double> lqr(a, b, q_, r_);
    if (!lqr.computeK())
    {
      ROS_ERROR("Failed to compute K of LQR.");
    }
    Eigen::Matrix<double, CONTROL_DIM, STATE_DIM> k = lqr.getK();
    ks.push_back(k);
  }
  polyfit(ks, lengths, coeffs_);

  Matrix<double, 2, 6> k;
  k.setZero();

  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 6; ++j)
    {
      k(i, j) = coeffs_(0, i + 2 * j) * pow(0.2, 3) + coeffs_(1, i + 2 * j) * pow(0.2, 2) +
                coeffs_(2, i + 2 * j) * 0.2 + coeffs_(3, i + 2 * j);
    }
  }
  std::cout << "len: 0.2m LQR k: " << std::endl << k << std::endl;

  bias_params_->x = config.x_bias;
  bias_params_->theta = config.theta_bias;
  bias_params_->raw_theta = config.raw_theta_bias;
}

double BipedalController::f_spring_force(double L0)
{
  //  static double l1 = leg_state_[LEFT].vmc->getL1(), l2 = leg_state_[LEFT].vmc->getL2();
  //  static double Fs = spring_params_->f_spring, s2 = spring_params_->s2, s3 = spring_params_->s3,
  //                alpha_s = spring_params_->alpha_s;
  //  double cos_theta3, theta3, ls, Fv;
  //  cos_theta3 = (l1 * l1 + l2 * l2 - L0 * L0) / (2 * l1 * l2);
  //  theta3 = acos(cos_theta3);
  //  ls = sqrt(s2 * s2 + s3 * s3 - 2 * s2 * s3 * cos(theta3 - alpha_s));
  //  Fv = Fs * (L0 * s2 * s3 * sin(theta3 - alpha_s)) / (ls * l1 * l2 * sin(theta3));
  //  return Fv;

  return ((2094.45f * L0 - 3091.28f) * L0 + 1408.375f) * L0 - 80.91f;
}

}  // namespace rm_chassis_controllers
PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BipedalController, controller_interface::ControllerBase)
