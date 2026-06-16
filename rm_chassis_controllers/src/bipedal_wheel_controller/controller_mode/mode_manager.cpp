//
// Created by guanlin on 25-9-4.
//

#include "bipedal_wheel_controller/controller_mode/mode_manager.h"

namespace rm_chassis_controllers
{
ModeManager::ModeManager(BipedalControllerInterface* controller, ros::NodeHandle& controller_nh,
                         const std::vector<hardware_interface::JointHandle*>& joint_handles)
{
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
  /*****************************************************************************************************/
  /* 这里涉及的所有 PID 控制器会被封装进一个wrapper
  /* 考虑到 PID 在逻辑上是共通的，我们这里的设计思路是在各个与 ROS 无关的模式（Mode）内部，将涉及到 PID 的部分全部解构
  /* 为调用了一个 PID 封装类的 computeCommand 方法，输入是 error 和 period、
  /* 这样类似的 wrapper 我们可以统一写成 ROSwrapper.h
  /* 报错信息也是同理，依赖注入会是我们解耦的一个重要技术
  /* 为此我们需要一个虚拟基类，包含
  /*****************************************************************************************************/
  for (const auto& e : pids)
    if (controller_nh.hasParam(e.first) && !e.second->init(ros::NodeHandle(controller_nh, e.first)))
      ROS_ERROR("Failed to load pid %s", e.first);
  pid_wheels_.push_back(&pid_left_wheel_vel_);
  pid_wheels_.push_back(&pid_right_wheel_vel_);
  pid_legs_.push_back(&pid_left_leg_);
  pid_legs_.push_back(&pid_right_leg_);
  pid_thetas_.push_back(&pid_left_leg_theta_);
  pid_thetas_.push_back(&pid_right_leg_theta_);
  pid_thetas_.push_back(&pid_left_leg_theta_vel_);
  pid_thetas_.push_back(&pid_right_leg_theta_vel_);
  pid_legs_stand_up_.push_back(&pid_left_leg_stand_up_);
  pid_legs_stand_up_.push_back(&pid_right_leg_stand_up_);

  mode_map_.insert(std::make_pair(BalanceMode::NORMAL,
                                  std::make_unique<Normal>(controller, joint_handles, pid_legs_, &pid_yaw_vel_,
                                                           &pid_theta_diff_, &pid_roll_, &pid_wheel_vel_diff_)));
  mode_map_.insert(std::make_pair(BalanceMode::STAND_UP, std::make_unique<StandUp>(controller, joint_handles,
                                                                                   pid_legs_stand_up_, pid_thetas_)));
  mode_map_.insert(
      std::make_pair(BalanceMode::RECOVER, std::make_unique<Recover>(controller, joint_handles, pid_legs_stand_up_,
                                                                     pid_thetas_, &pid_theta_diff_)));
  mode_map_.insert(
      std::make_pair(BalanceMode::SIT_DOWN, std::make_unique<SitDown>(controller, joint_handles, pid_wheels_)));
  mode_map_.insert(std::make_pair(BalanceMode::UPSTAIRS, std::make_unique<Upstairs>(controller, joint_handles,
                                                                                    pid_legs_stand_up_, pid_thetas_)));
  mode_map_.insert(std::make_pair(BalanceMode::UPSTAIRS, std::make_unique<Upstairs>(controller, joint_handles,
                                                                                    pid_legs_stand_up_, pid_thetas_)));
  mode_map_.insert(
      std::make_pair(BalanceMode::PROTECT, std::make_unique<Protect>(controller, joint_handles, pid_legs_, pid_thetas_,
                                                                     pid_wheels_, &pid_theta_diff_, &pid_yaw_vel_)));
}
}  // namespace rm_chassis_controllers
