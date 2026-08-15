//
// Created by Nesc on 26-6-16
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <rm_common/filters/filters.h>
#include <rm_common/lqr.h>
#include "rm_chassis_controllers/chassis_base.h"
#include "bipedal_wheel/core/core_types.h"
#include "bipedal_wheel/core/bipedal_wheel_core.h"
#include "ros/node_handle.h"

#include <geometry_msgs/Vector3.h>
#include <realtime_tools/realtime_publisher.h>
#include <rm_chassis_controllers/LQRWeightConfig.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Bool.h>
#include <rm_msgs/LeggedChassisStatus.h>
#include <rm_msgs/LeggedChassisMode.h>
#include <rm_msgs/LeggedLQRStatus.h>
#include <rm_msgs/LeggedUpstairStatus.h>
#include <rm_common/DebugDataPublisher.h>

namespace rm_chassis_controllers
{

class RosPidWrapper
{
public:
  RosPidWrapper() = default;
  explicit RosPidWrapper(control_toolbox::Pid* pid) : pid_(pid)
  {}

  double computeCommand(double error, double dt)
  {
    if (!pid_)
      return 0.0;
    // First call after (re)start: suppress the derivative kick (control_toolbox would
    // compute d_term = d * error/dt against a zero-initialized error_dot, which slams
    // the closed-chain joints against their loop constraints and starts a ringing
    // limit cycle).
    if (first_call_)
    {
      first_call_ = false;
      return pid_->computeCommand(error, 0.0, ros::Duration(dt));
    }
    return pid_->computeCommand(error, ros::Duration(dt));
  }

  void setPid(control_toolbox::Pid* pid)
  { pid_ = pid; }

private:
  control_toolbox::Pid* pid_ = nullptr;
  bool first_call_ = true;
};

class RosLoggerWrapper
{
public:
  void init(ros::NodeHandle& nh)
  {
    power_pub_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>>(nh, "power_debug", 1);
    unstick_pub_ = nh.advertise<std_msgs::Bool>("unstick", 1);
    upstair_status_pub_ = nh.advertise<rm_msgs::LeggedUpstairStatus>("upstair_status", 1);
    legged_chassis_status_pub_ = std::make_shared<realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisStatus>>(nh, "legged_chassis_status", 1);
    legged_chassis_mode_pub_ = std::make_shared<realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisMode>>(nh, "legged_chassis_mode", 1);
    lqr_status_pub_ = std::make_shared<realtime_tools::RealtimePublisher<rm_msgs::LeggedLQRStatus>>(nh, "lqr_status", 1);
    debugPub_ = std::make_shared<DebugDataPublisher>(nh, "debug_data");
  }
  void info(const std::string& msg)
  { ROS_INFO_STREAM(msg); }
  void warn(const std::string& msg)
  { ROS_WARN_STREAM(msg); }
  void error(const std::string& msg)
  { ROS_ERROR_STREAM(msg); }
  void publishPower(double power, double power_limit, double k_scale = 1.0)
  {
    if (power_pub_ && power_pub_->trylock())
    {
      power_pub_->msg_.x = power;
      power_pub_->msg_.y = power_limit;
      power_pub_->msg_.z = k_scale;
      power_pub_->unlockAndPublish();
    }
  }

  void publishLqrStatus(const Eigen::Matrix<double, 6, 1>& left_error,
                        const Eigen::Matrix<double, 6, 1>& right_error,
                        const Eigen::Matrix<double, 6, 1>& left_ref,
                        const Eigen::Matrix<double, 6, 1>& right_ref,
                        const Eigen::Matrix<double, 2, 1>& u_left,
                        const Eigen::Matrix<double, 2, 1>& u_right,
                        const Eigen::Vector2d& F_leg, const bool unstick[2])
  {
    if (lqr_status_pub_ && lqr_status_pub_->trylock())
    {
      lqr_status_pub_->msg_.left_leg_error.clear();
      lqr_status_pub_->msg_.right_leg_error.clear();
      lqr_status_pub_->msg_.left_leg_ref.clear();
      lqr_status_pub_->msg_.right_leg_ref.clear();
      for (int i = 0; i < 6; ++i)
      {
        lqr_status_pub_->msg_.left_leg_error.push_back(left_error(i));
        lqr_status_pub_->msg_.right_leg_error.push_back(right_error(i));
        lqr_status_pub_->msg_.left_leg_ref.push_back(left_ref(i));
        lqr_status_pub_->msg_.right_leg_ref.push_back(right_ref(i));
      }
      lqr_status_pub_->msg_.left_leg_u.clear();
      lqr_status_pub_->msg_.right_leg_u.clear();
      lqr_status_pub_->msg_.F_leg.clear();
      lqr_status_pub_->msg_.unstick.clear();
      for (int i = 0; i < 2; ++i)
      {
        lqr_status_pub_->msg_.left_leg_u.push_back(u_left(i));
        lqr_status_pub_->msg_.right_leg_u.push_back(u_right(i));
        lqr_status_pub_->msg_.F_leg.push_back(F_leg[i]);
        lqr_status_pub_->msg_.unstick.push_back(unstick[i]);
      }
      lqr_status_pub_->unlockAndPublish();
    }
  }

  void publishChassisStatus(double roll, double pitch, double d_pitch, double yaw, double d_yaw,
                            double left_leg_length, double right_leg_length, double x, double x_dot,
                            double left_leg_theta, double left_leg_theta_dot,
                            double right_leg_theta, double right_leg_theta_dot,
                            const geometry_msgs::Vector3& linear_acc_base)
  {
    if (legged_chassis_status_pub_ && legged_chassis_status_pub_->trylock())
    {
      legged_chassis_status_pub_->msg_.linear_acc_base.clear();
      legged_chassis_status_pub_->msg_.roll = roll;
      legged_chassis_status_pub_->msg_.pitch = pitch;
      legged_chassis_status_pub_->msg_.d_pitch = d_pitch;
      legged_chassis_status_pub_->msg_.yaw = yaw;
      legged_chassis_status_pub_->msg_.d_yaw = d_yaw;
      legged_chassis_status_pub_->msg_.left_leg_length = left_leg_length;
      legged_chassis_status_pub_->msg_.right_leg_length = right_leg_length;
      legged_chassis_status_pub_->msg_.x = x;
      legged_chassis_status_pub_->msg_.x_dot = x_dot;
      legged_chassis_status_pub_->msg_.left_leg_theta = left_leg_theta;
      legged_chassis_status_pub_->msg_.left_leg_theta_dot = left_leg_theta_dot;
      legged_chassis_status_pub_->msg_.right_leg_theta = right_leg_theta;
      legged_chassis_status_pub_->msg_.right_leg_theta_dot = right_leg_theta_dot;
      legged_chassis_status_pub_->msg_.linear_acc_base.push_back(linear_acc_base.x);
      legged_chassis_status_pub_->msg_.linear_acc_base.push_back(linear_acc_base.y);
      legged_chassis_status_pub_->msg_.linear_acc_base.push_back(linear_acc_base.z);
      legged_chassis_status_pub_->unlockAndPublish();
    }
  }

  void publishChassisMode(int mode, const std::string& mode_name)
  {
    if (legged_chassis_mode_pub_ && legged_chassis_mode_pub_->trylock())
    {
      legged_chassis_mode_pub_->msg_.mode = mode;
      legged_chassis_mode_pub_->msg_.mode_name = mode_name;
      legged_chassis_mode_pub_->unlockAndPublish();
    }
  }

  void publishUnstick(bool unstick)
  {
    std_msgs::Bool msg;
    msg.data = unstick;
    unstick_pub_.publish(msg);
  }

  void publishUpstairStatus(bool upstair_flag)
  {
    rm_msgs::LeggedUpstairStatus msg;
    msg.upstair_flag = upstair_flag;
    upstair_status_pub_.publish(msg);
  }

  void publishDebugData(const std::string& name, double value)
  {
    if (debugPub_)
      debugPub_->add(name, value);
  }

  void publishDebug()
  {
    if (debugPub_)
      debugPub_->publish();
  }

private:
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> power_pub_;
  ros::Publisher unstick_pub_, upstair_status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisStatus>> legged_chassis_status_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LeggedChassisMode>> legged_chassis_mode_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<rm_msgs::LeggedLQRStatus>> lqr_status_pub_;
  std::shared_ptr<DebugDataPublisher> debugPub_;
};

class BipedalController : public ChassisBase<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface,
                                             hardware_interface::EffortJointInterface>
{
public:
  BipedalController() = default;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void moveJoint(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;
  geometry_msgs::Twist odometry() override;

  // init Methods which use in init
  bool initParams(ros::NodeHandle& controller_nh);

  bool initRosInterface(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);

  bool initHardwareHandles(hardware_interface::RobotHW* robot_hw);

  bool initCoreAlgorithm(ros::NodeHandle& controller_nh);

  void lqrReconfigCB(rm_chassis_controllers::LQRWeightConfig& config, uint32_t level);

  // setup Methods which use while initParams
  bool setupLQR(ros::NodeHandle& controller_nh);
  bool setupModelParams(ros::NodeHandle& controller_nh);
  bool setupControlParams(ros::NodeHandle& controller_nh);
  bool setupBiasParams(ros::NodeHandle& controller_nh);
  bool setupThresholdParams(ros::NodeHandle& controller_nh);
  bool setupSpringParams(ros::NodeHandle& controller_nh);
  bool setupChassisGeometryParams(ros::NodeHandle& controller_nh);

  void
  polyfit(const std::vector<Eigen::Matrix<double, bipedal_wheel_core::CONTROL_DIM, bipedal_wheel_core::STATE_DIM>>& Ks,
          const std::vector<double>& L0s, Eigen::Matrix<double, 4, 12>& coeffs);

  // handles
  hardware_interface::ImuSensorHandle imu_handle_, gimbal_imu_handle_;
  hardware_interface::JointHandle left_wheel_joint_handle_, right_wheel_joint_handle_;
  hardware_interface::JointHandle left_hip_joint_handle_, left_knee_joint_handle_, right_hip_joint_handle_,
      right_knee_joint_handle_;
  std::vector<hardware_interface::JointHandle*> joint_handles_;

  // Params
  std::shared_ptr<bipedal_wheel_core::LqrModelParams> model_params_;
  std::shared_ptr<bipedal_wheel_core::ControlParams> control_params_;
  std::shared_ptr<bipedal_wheel_core::BiasParams> bias_params_;
  std::shared_ptr<bipedal_wheel_core::SpringParams> spring_params_;
  std::shared_ptr<bipedal_wheel_core::ChassisGeometryParams> chassis_geometry_params_;
  std::shared_ptr<bipedal_wheel_core::LegStateThresholdParams> leg_threshold_params_;
  double default_leg_length_ = 0.12;
  bool five_link_ = false;  // true: standard five-bar, hip raw angle needs +M_PI instead of +M_PI_2
  double knee_offset_ = -1.5707963267948966;  // added to raw knee handle to get VMC phi4 (fitted for linkage-driven legs)
  double hip_offset_ = 0.0;  // added to raw hip handle to get VMC phi1; unset (<-100) -> five_link ? M_PI : M_PI_2

  // TF cache for IMU <-> base_link static transform
  bool is_tf_cached_ = false;
  tf2::Transform imu2base_tf_;  // IMU -> base_link
  tf2::Transform base2imu_tf_;  // base_link -> IMU

  // LQR weights & coefficients
  Eigen::Matrix<double, bipedal_wheel_core::STATE_DIM, bipedal_wheel_core::STATE_DIM> q_;
  Eigen::Matrix<double, bipedal_wheel_core::CONTROL_DIM, bipedal_wheel_core::CONTROL_DIM> r_;
  Eigen::Matrix<double, 4, 12> coeffs_ = Eigen::Matrix<double, 4, 12>::Zero();
  dynamic_reconfigure::Server<rm_chassis_controllers::LQRWeightConfig>* lqr_reconfig_srv_ = nullptr;
  bool lqr_reconfig_initialized_ = false;

  // Control modes
  int balance_mode_ = 0;
  bool balance_state_changed_ = false;
  bool complete_stand_ = false;
  bool overturn_ = false;
  bool recovery_leg_spd_turnback_ = false;
  double last_yaw_vel_ = 0.0;

  // Subscriber commands
  double leg_length_cmd_ = 0.12;
  bool jump_cmd_ = false;
  ros::Subscriber leg_cmd_sub_;
  ros::Subscriber recovery_leg_spd_turnback_sub_;

  // PID controllers (raw control_toolbox::Pid objects)
  control_toolbox::Pid pid_yaw_vel_, pid_theta_diff_, pid_roll_, pid_wheel_vel_diff_;
  control_toolbox::Pid pid_left_leg_, pid_right_leg_;
  control_toolbox::Pid pid_left_leg_stand_up_, pid_right_leg_stand_up_;
  control_toolbox::Pid pid_left_leg_theta_, pid_right_leg_theta_;
  control_toolbox::Pid pid_left_leg_theta_vel_, pid_right_leg_theta_vel_;
  control_toolbox::Pid pid_left_wheel_vel_, pid_right_wheel_vel_;

  // PID wrappers
  RosPidWrapper pid_wrapper_yaw_vel_, pid_wrapper_theta_diff_;
  RosPidWrapper pid_wrapper_roll_, pid_wrapper_wheel_vel_diff_;
  RosPidWrapper pid_wrapper_left_leg_, pid_wrapper_right_leg_;
  RosPidWrapper pid_wrapper_left_leg_stand_up_, pid_wrapper_right_leg_stand_up_;
  RosPidWrapper pid_wrapper_left_leg_theta_, pid_wrapper_right_leg_theta_;
  RosPidWrapper pid_wrapper_left_leg_theta_vel_, pid_wrapper_right_leg_theta_vel_;
  RosPidWrapper pid_wrapper_left_wheel_vel_, pid_wrapper_right_wheel_vel_;

  // Wrapper pointer vectors passed into core_.init()
  std::vector<RosPidWrapper*> pid_wrappers_legs_;
  std::vector<RosPidWrapper*> pid_wrappers_legs_stand_up_;
  std::vector<RosPidWrapper*> pid_wrappers_thetas_;
  std::vector<RosPidWrapper*> pid_wrappers_wheels_;

  // Logger wrapper
  RosLoggerWrapper logger_;

  // Core algorithm instance, inject PID and logger wrapper inside
  bipedal_wheel_core::BipedalWheelCore<RosPidWrapper, RosLoggerWrapper, RampFilter<double>, MovingAverageFilter<double>>
      core_;
};

}  // namespace rm_chassis_controllers