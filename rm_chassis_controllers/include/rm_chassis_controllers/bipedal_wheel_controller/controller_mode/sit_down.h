//
// Created by guanlin on 25-9-3.
//

#pragma once

#include <memory>
#include <vector>

#include "bipedal_wheel_controller/controller_mode/mode_base.h"
#include "bipedal_wheel_controller/pid_interface.h"

namespace rm_chassis_controllers
{
class SitDown : public ModeBase
{
public:
  explicit SitDown(PidInterface* left_wheel, PidInterface* right_wheel);
  LogicOutput execute(const LogicInput& input) override;
  const char* name() const override
  {
    return "SIT_DOWN";
  }

private:
  PidInterface *pid_left_wheel_, *pid_right_wheel_;
};
}  // namespace rm_chassis_controllers
