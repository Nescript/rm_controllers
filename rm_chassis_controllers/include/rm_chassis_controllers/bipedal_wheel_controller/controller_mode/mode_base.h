//
// Created by guanlin on 25-9-3.
//

#pragma once

#include "bipedal_wheel_controller/logic_types.h"

namespace rm_chassis_controllers
{
class ModeBase
{
public:
  virtual ~ModeBase() = default;
  virtual LogicOutput execute(const LogicInput& input) = 0;
  virtual const char* name() const = 0;
  void updateUnstick(const bool& left_unstick, const bool& right_unstick);

  inline bool getUnstick()
  {
    return (left_unstick_ && right_unstick_);
  }

protected:
  bool left_unstick_{ false }, right_unstick_{ false };
};

}  // namespace rm_chassis_controllers
