#pragma once
#include "bipedal_wheel/core/fsm/fsm_context.h"

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class SitDown
{
public:
  SitDown() = default;

  void execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx);
};

}  // namespace bipedal_wheel_core
