//
// Created by Nesc on 26-6-16
//

#include "rm_chassis_controllers/bipedal_wheel/controllers.h"

namespace rm_chassis_controllers
{
bool BipedalController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  ChassisBase::init(robot_hw, root_nh, controller_nh);

  if (!initParams(controller_nh))
    return false;

  initRosInterface(root_nh, controller_nh);

  if (!initHardwareHandles(robot_hw))
  {
    ROS_ERROR("[BIPED_WHEEL_CHASSIS] Failed to extract hardware handles");
    // 考虑进一步的错误处理，在职责函数内部进行
    return false;
  }
  if (initCoreAlgorithm())
  {
    return false;
  }
  return true;
}

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

/*
bool BipedalController::initParams(ros::NodeHandle& controller_nh)
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
  if (!controller_nh.getParam("jumpOverTime", control_params_->jumpOverTime_))
  {
    ROS_ERROR("Load param fail, check the resist of jump_over_time");
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
*/
};
}  // namespace rm_chassis_controllers