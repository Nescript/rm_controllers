//
// Created by guanlin on 25-9-4.
//

#include "bipedal_wheel_controller/controller_mode/mode_manager.h"
#include "bipedal_wheel_controller/controller_mode/normal.h"
#include "bipedal_wheel_controller/controller_mode/stand_up.h"
#include "bipedal_wheel_controller/controller_mode/recover.h"
#include "bipedal_wheel_controller/controller_mode/sit_down.h"
#include "bipedal_wheel_controller/controller_mode/upstairs.h"
#include "bipedal_wheel_controller/controller_mode/protect.h"

namespace rm_chassis_controllers
{
ModeManager::ModeManager(const PidSet& pids)
{
  mode_map_.insert(std::make_pair(BalanceMode::NORMAL,
                                  std::make_shared<Normal>(pids.left_leg, pids.right_leg, pids.yaw_vel,
                                                           pids.theta_diff, pids.roll, pids.wheel_vel_diff)));
  mode_map_.insert(std::make_pair(BalanceMode::STAND_UP,
                                  std::make_shared<StandUp>(pids.left_leg_stand_up, pids.right_leg_stand_up,
                                                            pids.left_leg_theta, pids.right_leg_theta,
                                                            pids.left_leg_theta_vel, pids.right_leg_theta_vel)));
  mode_map_.insert(std::make_pair(BalanceMode::RECOVER,
                                  std::make_shared<Recover>(pids.left_leg_stand_up, pids.right_leg_stand_up,
                                                            pids.left_leg_theta, pids.right_leg_theta,
                                                            pids.theta_diff)));
  mode_map_.insert(std::make_pair(BalanceMode::SIT_DOWN,
                                  std::make_shared<SitDown>(pids.left_wheel_vel, pids.right_wheel_vel)));
  mode_map_.insert(std::make_pair(BalanceMode::UPSTAIRS,
                                  std::make_shared<Upstairs>(pids.left_leg_stand_up, pids.right_leg_stand_up,
                                                             pids.left_leg_theta, pids.right_leg_theta,
                                                             pids.left_leg_theta_vel, pids.right_leg_theta_vel)));
  mode_map_.insert(std::make_pair(BalanceMode::PROTECT,
                                  std::make_shared<Protect>(pids.left_leg, pids.right_leg, pids.left_leg_theta,
                                                            pids.right_leg_theta, pids.left_wheel_vel,
                                                            pids.right_wheel_vel, pids.theta_diff, pids.yaw_vel)));
}
}  // namespace rm_chassis_controllers
