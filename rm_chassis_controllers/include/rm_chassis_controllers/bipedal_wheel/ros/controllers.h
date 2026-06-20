//
// Created by Nesc on 26-6-16
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include "rm_chassis_controllers/chassis_base.h"
#include "bipedal_wheel/core/bipedal_wheel_core.h"
#include "bipedal_wheel/core/definitions.h"

namespace rm_chassis_controllers
{

class BipedalController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                                             hardware_interface::EffortJointInterface>
{
public:
  BipedalController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

  // init Methods which use in init
  bool initParams(ros::NodeHandle& controller_nh);

  bool initRosInterface(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  bool initHardwareHandles(hardware_interface::RobotHW* robot_hw);

  bool initCoreAlgorithm();

  // setup Methods which use while initParams
  bool setupLQR(ros::NodeHandle& controller_nh);
  bool setupModelParams(ros::NodeHandle& controller_nh);
  bool setupControlParams(ros::NodeHandle& controller_nh);
  bool setupBiasParams(ros::NodeHandle& controller_nh);
  bool setupThresholdParams(ros::NodeHandle& controller_nh);
  bool setupSpringParams(ros::NodeHandle& controller_nh);
  bool setupChassisGeometryParams(ros::NodeHandle& controller_nh);

private:
  // Core algorithm library instance
  bipedal_wheel_core::BipedalWheelCore core_;

  // Static transforms to be cached and injected
  Eigen::Quaterniond q_base2imu_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond q_imu2base_ = Eigen::Quaterniond::Identity();

  // VMC parameters
  double l1_ = 0.0;
  double l2_ = 0.0;
  double default_leg_length_ = 0.12;

  // handles
  hardware_interface::ImuSensorHandle imu_handle_, gimbal_imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_;
  hardware_interface::JointHandle left_hip_joint_handle_, left_knee_joint_handle_, right_hip_joint_handle_,
      right_knee_joint_handle_;
  std::vector<hardware_interface::JointHandle*> joint_handles_;

  // Params
  std::shared_ptr<bipedal_wheel_core::ModelParams> model_params_;
  std::shared_ptr<bipedal_wheel_core::ControlParams> control_params_;
  std::shared_ptr<bipedal_wheel_core::BiasParams> bias_params_;
  std::shared_ptr<bipedal_wheel_core::SpringParams> spring_params_;
  std::shared_ptr<bipedal_wheel_core::ChassisGeometryParams> chassis_geometry_params_;
  std::shared_ptr<bipedal_wheel_core::LegStateThresholdParams> leg_threshold_params_;
};

}  // namespace rm_chassis_controllers