//
// Created by guanlin on 25-9-4.
//

#pragma once

#include <map>
#include <memory>
#include <vector>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/pid_interface.h"

namespace rm_chassis_controllers
{
class ModeManager
{
public:
  ModeManager(const PidSet& pids);
  virtual ~ModeManager() = default;
  
  void switchMode(int mode)
  {
    mode_impl = mode_map_[mode];
  }
  
  const std::shared_ptr<ModeBase>& getModeImpl()
  {
    return mode_impl;
  }
  
  LogicOutput execute(const LogicInput& input)
  {
    return mode_impl->execute(input);
  }

private:
  std::shared_ptr<ModeBase> mode_impl;
  std::map<int, std::shared_ptr<ModeBase>> mode_map_;
};
}  // namespace rm_chassis_controllers
