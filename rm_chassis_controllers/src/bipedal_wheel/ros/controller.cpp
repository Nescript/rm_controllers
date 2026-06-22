//
// Created by Nesc on 26-6-16
//

#include "rm_chassis_controllers/bipedal_wheel/controllers.h"
#include "ros/node_handle.h"
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/Bool.h>
#include <rm_msgs/LegCmd.h>

namespace rm_chassis_controllers
{
bool BipedalController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
              hardware_interface::EffortJointInterface>::init(robot_hw, root_nh, controller_nh);

  if (!initParams(controller_nh))
    return false;

  initRosInterface(root_nh, controller_nh);

  if (!initHardwareHandles(robot_hw))
  {
    return false;
  }

  if (!initCoreAlgorithm())
  {
    return false;
  }

  // Cache static TF at init time if available
  try
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), ros::Time(0));
    tf2::fromMsg(tf_msg.transform, imu2base_tf_);
    base2imu_tf_ = imu2base_tf_.inverse();
    is_tf_cached_ = true;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("[BipedalController] Static TF IMU->base_link not available at init, will lookup lazily: %s", ex.what());
  }

  return true;
}

/********************************** 应该移动到 .h 文件 ***********************************/
inline void quatToRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
  double as = std::min(-2. * (q.x * q.z - q.w * q.y), .99999);
  yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
  pitch = std::asin(as);
  roll = std::atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
}

inline void setJointCommands(std::vector<hardware_interface::JointHandle*>& joints,
                             const bipedal_wheel_core::ControlOutput& ctrl_out)
{
  if (joints.size() != 6)
    throw std::runtime_error("Joint handle vector size must be 6!");

  // 关节顺序在 initHardwareHandles 中初始化为:
  // left_hip, left_knee, right_hip, right_knee, left_wheel, right_wheel
  joints[0]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::LEFT].hip.effort);
  joints[1]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::LEFT].knee.effort);
  joints[2]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::RIGHT].hip.effort);
  joints[3]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::RIGHT].knee.effort);
  joints[4]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::LEFT].wheel.effort);
  joints[5]->setCommand(ctrl_out.leg_cmd[bipedal_wheel_core::RIGHT].wheel.effort);
}

void BipedalController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
  // Check state overrides for balance mode transitions
  if (state_ == rm_msgs::ChassisCmd::FALLEN)
  {
    core_.setRobotMode(bipedal_wheel_core::RobotPhysicalState::FALLEN);
    core_.setBalanceStateChanged(false);
  }
  else if (!overturn_ && state_ == rm_msgs::ChassisCmd::RECOVERY)
  {
    overturn_ = true;
    core_.setRobotMode(bipedal_wheel_core::RobotPhysicalState::GETTING_UP);
    core_.setBalanceStateChanged(false);
  }

  // get info from imu
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

  bipedal_wheel_core::SensorMeasurements sens_in{};

  if (!is_tf_cached_)
  {
    try
    {
      geometry_msgs::TransformStamped tf_msg;
      tf_msg = robot_state_handle_.lookupTransform("base_link", imu_handle_.getFrameId(), ros::Time(0));
      tf2::fromMsg(tf_msg.transform, imu2base_tf_);
      base2imu_tf_ = imu2base_tf_.inverse();
      is_tf_cached_ = true;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_ONCE("[BipedalController] Static TF IMU->base_link not available yet: %s", ex.what());
      bipedal_wheel_core::ControlOutput zero_cmd{};
      setJointCommands(joint_handles_, zero_cmd);
      last_yaw_vel_ = 0.0;
      return;
    }
  }

  try
  {
    // 将 imu 数据变换到 base link 坐标系
    tf2::Vector3 gyro_tf2(gyro.x, gyro.y, gyro.z);
    tf2::Vector3 angular_vel_base_tf2 = imu2base_tf_ * gyro_tf2;
    angular_vel_base.x = angular_vel_base_tf2.x();
    angular_vel_base.y = angular_vel_base_tf2.y();
    angular_vel_base.z = angular_vel_base_tf2.z();

    // get imu to base transform
    imu2base = base2imu_tf_;

    // get odom to imu transform
    tf2::Quaternion odom2imu_quaternion;
    tf2::Vector3 odom2imu_origin;
    odom2imu_quaternion.setValue(imu_handle_.getOrientation()[0], imu_handle_.getOrientation()[1],
                                 imu_handle_.getOrientation()[2], imu_handle_.getOrientation()[3]);
    odom2imu_origin.setValue(0, 0, 0);
    odom2imu.setOrigin(odom2imu_origin);
    odom2imu.setRotation(odom2imu_quaternion);
    odom2base = odom2imu * imu2base;
    quatToRPY(toMsg(odom2base).rotation, roll, pitch, yaw);

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.transform = tf2::toMsg(odom2imu.inverse());
    tf_msg.header.stamp = time;
    tf2::doTransform(acc, linear_acc_base, tf_msg);

    // 打包底盘传感器数据
    sens_in.chassis_state.angular_vel << angular_vel_base.x, angular_vel_base.y, angular_vel_base.z;
    sens_in.chassis_state.linear_acc << linear_acc_base.x, linear_acc_base.y, linear_acc_base.z;
    sens_in.chassis_state.roll = roll;
    sens_in.chassis_state.pitch = pitch;
    sens_in.chassis_state.yaw = yaw;
    last_yaw_vel_ = angular_vel_base.z;
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_ONCE("%s", ex.what());
    bipedal_wheel_core::ControlOutput zero_cmd{};
    setJointCommands(joint_handles_, zero_cmd);
    last_yaw_vel_ = 0.0;
    return;
  }

  // 2. 打包左腿关节状态 (hip, knee, wheel)
  sens_in.leg_state[bipedal_wheel_core::LEFT].hip.position = left_hip_joint_handle_.getPosition() + M_PI_2;
  sens_in.leg_state[bipedal_wheel_core::LEFT].hip.vel = left_hip_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::LEFT].hip.effort = left_hip_joint_handle_.getEffort();

  sens_in.leg_state[bipedal_wheel_core::LEFT].knee.position = left_knee_joint_handle_.getPosition() - M_PI_2;
  sens_in.leg_state[bipedal_wheel_core::LEFT].knee.vel = left_knee_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::LEFT].knee.effort = left_knee_joint_handle_.getEffort();

  sens_in.leg_state[bipedal_wheel_core::LEFT].wheel.position = left_wheel_joint_handle_.getPosition();
  sens_in.leg_state[bipedal_wheel_core::LEFT].wheel.vel = left_wheel_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::LEFT].wheel.effort = left_wheel_joint_handle_.getEffort();

  // 3. 打包右腿关节状态 (hip, knee, wheel)
  sens_in.leg_state[bipedal_wheel_core::RIGHT].hip.position = right_hip_joint_handle_.getPosition() + M_PI_2;
  sens_in.leg_state[bipedal_wheel_core::RIGHT].hip.vel = right_hip_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::RIGHT].hip.effort = right_hip_joint_handle_.getEffort();

  sens_in.leg_state[bipedal_wheel_core::RIGHT].knee.position = right_knee_joint_handle_.getPosition() - M_PI_2;
  sens_in.leg_state[bipedal_wheel_core::RIGHT].knee.vel = right_knee_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::RIGHT].knee.effort = right_knee_joint_handle_.getEffort();

  // 4. 打包右腿车轮状态
  sens_in.leg_state[bipedal_wheel_core::RIGHT].wheel.position = right_wheel_joint_handle_.getPosition();
  sens_in.leg_state[bipedal_wheel_core::RIGHT].wheel.vel = right_wheel_joint_handle_.getVelocity();
  sens_in.leg_state[bipedal_wheel_core::RIGHT].wheel.effort = right_wheel_joint_handle_.getEffort();

  // 5. 组装输入指令 commands
  bipedal_wheel_core::ControllerCommands cmd_in{};
  cmd_in.vel_cmd << vel_cmd_.x, vel_cmd_.y, vel_cmd_.z;
  cmd_in.leg_length_cmd = leg_length_cmd_;
  cmd_in.jump_cmd = jump_cmd_;
  cmd_in.overturn = overturn_;
  cmd_in.coeffs = &coeffs_;

  if (state_ == rm_msgs::ChassisCmd::RAW)
    cmd_in.base_state = 1;
  else if (state_ == rm_msgs::ChassisCmd::FOLLOW)
    cmd_in.base_state = 2;
  else if (state_ == rm_msgs::ChassisCmd::FALLEN)
    cmd_in.base_state = 3;
  else if (state_ == rm_msgs::ChassisCmd::RECOVERY)
    cmd_in.base_state = 4;
  else
    cmd_in.base_state = 0;

  // 6. 传入时间步长 dt 与传感器测量，运行核心算法
  double dt = period.toSec();
  core_.setCompleteStand(complete_stand_);
  core_.setOverturn(overturn_);
  core_.setBalanceStateChanged(balance_state_changed_);
  core_.setRecoveryLegSpdTurnback(recovery_leg_spd_turnback_);

  bipedal_wheel_core::ControlOutput ctrl_out = core_.update(dt, sens_in, cmd_in);

  // 7. 回收状态机运行结果
  complete_stand_ = core_.getCompleteStand();
  overturn_ = core_.getOverturn();
  balance_state_changed_ = core_.getBalanceStateChanged();
  recovery_leg_spd_turnback_ = core_.getRecoveryLegSpdTurnback();

  // 8. 将输出控制量应用给电机
  setJointCommands(joint_handles_, ctrl_out);
}

void BipedalController::stopping(const ros::Time& time)
{
  core_.setRobotMode(bipedal_wheel_core::RobotPhysicalState::HANGING);
  balance_state_changed_ = false;
  bipedal_wheel_core::ControlOutput zero_cmd{};
  setJointCommands(joint_handles_, zero_cmd);

  ROS_INFO("[balance] Controller Stop");
}

/**
 * @brief Init hardware interface and extract imu, joint handle from robot_hw ptr
 *
 * @param robot_hw robot hardware ptr
 * @return true successfully get all hardware handle
 * @return false failed to get hardware interface or didint find handle
 */
bool BipedalController::initHardwareHandles(hardware_interface::RobotHW* robot_hw)
{
  auto* imu_interface = robot_hw->get<hardware_interface::ImuSensorInterface>();
  if (imu_interface == nullptr)
  {
    ROS_ERROR("[BipedalController] Failed to get ImuSensorInterface.");
    return false;
  }
  try
  {
    imu_handle_ = imu_interface->getHandle("base_imu");
  }
  catch (const hardware_interface::HardwareInterfaceException& ex)
  {
    ROS_ERROR_STREAM("[BipedalController] Failed to get imu handle: " << ex.what());
    return false;
  }

  //  gimbal_imu_handle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("gimbal_imu");

  auto* joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (joint_interface == nullptr)
  {
    ROS_ERROR("[BipedalController] Failed to get EffortJointInterface.");
    return false;
  }

  const std::pair<const char*, hardware_interface::JointHandle*> table[] = {
    { "left_hip_joint", &left_hip_joint_handle_ },     { "left_knee_joint", &left_knee_joint_handle_ },
    { "right_hip_joint", &right_hip_joint_handle_ },   { "right_knee_joint", &right_knee_joint_handle_ },
    { "left_wheel_joint", &left_wheel_joint_handle_ }, { "right_wheel_joint", &right_wheel_joint_handle_ }
  };
  for (const auto& t : table)
  {
    try
    {
      *t.second = joint_interface->getHandle(t.first);
      joint_handles_.push_back(t.second);
    }
    catch (const hardware_interface::HardwareInterfaceException& ex)
    {
      ROS_ERROR_STREAM("[BipedalController] Failed to get joint handle for " << t.first << ": " << ex.what());
      return false;
    }
  }

  return true;
}

bool BipedalController::initRosInterface(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  auto legCmdCallback = [this](const rm_msgs::LegCmd::ConstPtr& msg) {
    leg_length_cmd_ = msg->leg_length;
    jump_cmd_ = msg->jump;
  };
  auto recoveryLegSpdTurnbackCb = [this](const std_msgs::Bool::ConstPtr& msg) {
    recovery_leg_spd_turnback_ = msg->data;
  };
  leg_cmd_sub_ = controller_nh.subscribe<rm_msgs::LegCmd>("/leg_cmd", 5, legCmdCallback);
  recovery_leg_spd_turnback_sub_ =
      controller_nh.subscribe<std_msgs::Bool>("/recovery_leg_spd_turnback", 1, recoveryLegSpdTurnbackCb);

  return true;
}

bool BipedalController::initParams(ros::NodeHandle& controller_nh)
{
  model_params_ = std::make_shared<bipedal_wheel_core::LqrModelParams>();
  control_params_ = std::make_shared<bipedal_wheel_core::ControlParams>();
  bias_params_ = std::make_shared<bipedal_wheel_core::BiasParams>();
  leg_threshold_params_ = std::make_shared<bipedal_wheel_core::LegStateThresholdParams>();
  spring_params_ = std::make_shared<bipedal_wheel_core::SpringParams>();
  chassis_geometry_params_ = std::make_shared<bipedal_wheel_core::ChassisGeometryParams>();

  // Load PIDs
  const std::pair<const char*, control_toolbox::Pid*> pids[] = {
    { "pid_yaw_vel", &pid_yaw_vel_ },
    { "pid_left_leg", &pid_left_leg_ },
    { "pid_right_leg", &pid_right_leg_ },
    { "pid_theta_diff", &pid_theta_diff_ },
    { "pid_roll", &pid_roll_ },
    { "pid_left_leg_theta", &pid_left_leg_theta_ },
    { "pid_right_leg_theta", &pid_right_leg_theta_ },
    { "pid_left_leg_theta_vel", &pid_left_leg_theta_vel_ },
    { "pid_right_leg_theta_vel", &pid_right_leg_theta_vel_ },
    { "pid_left_wheel_vel", &pid_left_wheel_vel_ },
    { "pid_right_wheel_vel", &pid_right_wheel_vel_ },
    { "pid_left_leg_stand_up", &pid_left_leg_stand_up_ },
    { "pid_right_leg_stand_up", &pid_right_leg_stand_up_ },
    { "pid_wheel_vel_diff", &pid_wheel_vel_diff_ },
  };

  for (const auto& e : pids)
  {
    if (controller_nh.hasParam(e.first) && !e.second->init(ros::NodeHandle(controller_nh, e.first)))
    {
      ROS_ERROR("Failed to load pid %s", e.first);
      return false;
    }
  }

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
  {
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  }

  if (!controller_nh.getParam("l1", model_params_->l1) || !controller_nh.getParam("l2", model_params_->l2))
  {
    ROS_ERROR("Param %s or %s not given (namespace: %s)", "l1", "l2", controller_nh.getNamespace().c_str());
    return false;
  }
  controller_nh.param("l5", model_params_->l5, 0.0);

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
  {
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
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
  Eigen::VectorXd q_diag = loadWeightMatrix(controller_nh, "q", bipedal_wheel_core::STATE_DIM);
  Eigen::VectorXd r_diag = loadWeightMatrix(controller_nh, "r", bipedal_wheel_core::CONTROL_DIM);
  if (!q_diag.allFinite() || !r_diag.allFinite())
    return false;
  q_.setZero();
  r_.setZero();
  q_.diagonal() = q_diag;
  r_.diagonal() = r_diag;

  // Continuous model \dot{x} = A x + B u
  std::vector<double> lengths;
  std::vector<Eigen::Matrix<double, bipedal_wheel_core::CONTROL_DIM, bipedal_wheel_core::STATE_DIM>> ks;
  for (int i = 10; i < 40; i++)
  {
    double length = i / 100.0f;
    lengths.push_back(length);
    Eigen::Matrix<double, bipedal_wheel_core::STATE_DIM, bipedal_wheel_core::STATE_DIM> a{};
    Eigen::Matrix<double, bipedal_wheel_core::STATE_DIM, bipedal_wheel_core::CONTROL_DIM> b{};
    generateAB(*model_params_, a, b, length);
    Lqr<double> lqr(a, b, q_, r_);
    if (!lqr.computeK())
    {
      ROS_ERROR("Failed to compute K of LQR.");
      return false;
    }
    Eigen::Matrix<double, bipedal_wheel_core::CONTROL_DIM, bipedal_wheel_core::STATE_DIM> k = lqr.getK();
    ks.push_back(k);
  }
  polyfit(ks, lengths, coeffs_);

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
  {
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  }
  return true;
}

// [will unused]
bool BipedalController::setupControlParams(ros::NodeHandle& controller_nh)
{
  if (!controller_nh.getParam("jumpOverTime", control_params_->jump_over_time))
  {
    ROS_ERROR("Load param fail, check the resist of jumpOverTime");
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
  {
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
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
  {
    if (!controller_nh.getParam(e.first, *e.second))
    {
      ROS_ERROR("Param %s not given (namespace: %s)", e.first, controller_nh.getNamespace().c_str());
      return false;
    }
  }
  return true;
}

bool BipedalController::initCoreAlgorithm()
{
  // 1. Initialize wrappers
  pid_wrapper_yaw_vel_.setPid(&pid_yaw_vel_);
  pid_wrapper_theta_diff_.setPid(&pid_theta_diff_);
  pid_wrapper_roll_.setPid(&pid_roll_);
  pid_wrapper_wheel_vel_diff_.setPid(&pid_wheel_vel_diff_);

  pid_wrapper_left_leg_.setPid(&pid_left_leg_);
  pid_wrapper_right_leg_.setPid(&pid_right_leg_);
  pid_wrapper_left_leg_stand_up_.setPid(&pid_left_leg_stand_up_);
  pid_wrapper_right_leg_stand_up_.setPid(&pid_right_leg_stand_up_);

  pid_wrapper_left_leg_theta_.setPid(&pid_left_leg_theta_);
  pid_wrapper_right_leg_theta_.setPid(&pid_right_leg_theta_);
  pid_wrapper_left_leg_theta_vel_.setPid(&pid_left_leg_theta_vel_);
  pid_wrapper_right_leg_theta_vel_.setPid(&pid_right_leg_theta_vel_);

  pid_wrapper_left_wheel_vel_.setPid(&pid_left_wheel_vel_);
  pid_wrapper_right_wheel_vel_.setPid(&pid_right_wheel_vel_);

  // 2. Populate wrapper vectors
  pid_wrappers_legs_ = { &pid_wrapper_left_leg_, &pid_wrapper_right_leg_ };
  pid_wrappers_legs_stand_up_ = { &pid_wrapper_left_leg_stand_up_, &pid_wrapper_right_leg_stand_up_ };

  pid_wrappers_thetas_ = { &pid_wrapper_left_leg_theta_, &pid_wrapper_right_leg_theta_,
                           &pid_wrapper_left_leg_theta_vel_, &pid_wrapper_right_leg_theta_vel_ };

  pid_wrappers_wheels_ = { &pid_wrapper_left_wheel_vel_, &pid_wrapper_right_wheel_vel_ };

  // 3. Assemble parameters
  bipedal_wheel_core::ControllerParams params{};
  if (model_params_)
    params.model_params = *model_params_;
  if (chassis_geometry_params_)
    params.chassis_geometry = *chassis_geometry_params_;
  if (spring_params_)
    params.spring = *spring_params_;
  if (control_params_)
    params.control = *control_params_;
  if (bias_params_)
    params.bias = *bias_params_;
  if (leg_threshold_params_)
    params.threshold = *leg_threshold_params_;
  params.default_leg_length = default_leg_length_;

  // 4. Initialize core algorithm
  if (!core_.init(params, logger_, pid_wrappers_legs_, pid_wrappers_legs_stand_up_, pid_wrappers_thetas_,
                  pid_wrappers_wheels_, &pid_wrapper_yaw_vel_, &pid_wrapper_theta_diff_, &pid_wrapper_roll_,
                  &pid_wrapper_wheel_vel_diff_))
  {
    ROS_ERROR("[BipedalController] Failed to initialize core algorithm.");
    return false;
  }
  return true;
}

void BipedalController::polyfit(
    const std::vector<Eigen::Matrix<double, bipedal_wheel_core::CONTROL_DIM, bipedal_wheel_core::STATE_DIM>>& Ks,
    const std::vector<double>& L0s, Eigen::Matrix<double, 4, 12>& coeffs)
{
  int N = L0s.size();
  Eigen::MatrixXd A(N, 4), B(N, 12);
  for (int i = 0; i < N; ++i)
  {
    A.block(i, 0, 1, 4) << std::pow(L0s[i], 3), std::pow(L0s[i], 2), L0s[i], 1.0;
    Eigen::Map<const Eigen::Matrix<double, 12, 1>> flat(Ks[i].data());
    B.row(i) = flat.transpose();
  }
  coeffs = (A.transpose() * A).ldlt().solve(A.transpose() * B);
}

geometry_msgs::Twist BipedalController::odometry()
{
  geometry_msgs::Twist twist;
  if (core_.getRobotMode() != bipedal_wheel_core::RobotPhysicalState::HANGING)
  {
    twist.linear.x = core_.getLqrStatus().dx;
    twist.angular.z = last_yaw_vel_;
  }
  else
  {
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
  }
  return twist;
}

}  // namespace rm_chassis_controllers

PLUGINLIB_EXPORT_CLASS(rm_chassis_controllers::BipedalController, controller_interface::ControllerBase)