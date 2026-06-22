//
// Created by Nesc on 26-6-16
//

#pragma once
#include "bipedal_wheel/core/core_types.h"
#include "bipedal_wheel/core/vmc/VMC.h"
#include "bipedal_wheel/core/fsm/state_sit_down.h"
#include "bipedal_wheel/core/fsm/state_stand_up.h"
#include "bipedal_wheel/core/fsm/state_normal.h"
#include "bipedal_wheel/core/fsm/state_recover.h"
#include "bipedal_wheel/core/fsm/state_upstairs.h"
#include "bipedal_wheel/core/fsm/state_protect.h"
#include <memory>
#include <variant>
#include <vector>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class BipedalWheelCore
{
public:
  BipedalWheelCore() = default;

  // Initialize all parameters, estimators, and visualizers
  bool init(const bipedal_wheel_core::ControllerParams& config, LoggerType& logger,
            const std::vector<PidType*>& pid_legs, const std::vector<PidType*>& pid_legs_stand_up,
            const std::vector<PidType*>& pid_thetas, const std::vector<PidType*>& pid_wheels, PidType* pid_yaw_vel,
            PidType* pid_theta_diff, PidType* pid_roll, PidType* pid_wheel_vel_diff);

  // Compute control command loop iteration
  bipedal_wheel_core::ControlOutput update(double dt, const bipedal_wheel_core::SensorMeasurements& sens_in,
                                           const bipedal_wheel_core::ControllerCommands& cmd_in);

  // Reset internal states and estimation filters
  void reset();

  // Expose status metrics for telemetry and visualization
  const bipedal_wheel_core::LQRStatus& getLqrStatus() const
  { return lqr_status_; }
  bipedal_wheel_core::RobotMode getRobotMode() const
  { return robot_mode_; }

  const VMC* getLeftVmc() const
  { return left_vmc_.get(); }
  const VMC* getRightVmc() const
  { return right_vmc_.get(); }

  // Check state machine control flags
  bool getCompleteStand() const
  { return complete_stand_; }
  bool getOverturn() const
  { return overturn_; }
  bool getBalanceStateChanged() const
  { return balance_state_changed_; }
  bool getRecoveryLegSpdTurnback() const
  { return recovery_leg_spd_turnback_; }

  void setCompleteStand(bool val)
  { complete_stand_ = val; }
  void setOverturn(bool val)
  { overturn_ = val; }
  void setBalanceStateChanged(bool val)
  { balance_state_changed_ = val; }
  void setRecoveryLegSpdTurnback(bool val)
  { recovery_leg_spd_turnback_ = val; }
  void setRobotMode(bipedal_wheel_core::RobotMode val)
  { robot_mode_ = val; }

private:
  bipedal_wheel_core::ControllerParams config_;
  bipedal_wheel_core::LQRStatus lqr_status_;
  bipedal_wheel_core::RobotMode robot_mode_ = bipedal_wheel_core::RobotMode::HANGING;

  std::unique_ptr<VMC> left_vmc_;
  std::unique_ptr<VMC> right_vmc_;

  // Shared internal flags
  bool complete_stand_ = false;
  bool overturn_ = false;
  bool move_flag_ = false;
  bool balance_state_changed_ = false;
  bool recovery_leg_spd_turnback_ = false;

  // Injected component pointers/references cached during initialization
  LoggerType* logger_ = nullptr;
  std::vector<PidType*> pid_legs_;
  std::vector<PidType*> pid_legs_stand_up_;
  std::vector<PidType*> pid_thetas_;
  std::vector<PidType*> pid_wheels_;
  PidType* pid_yaw_vel_ = nullptr;
  PidType* pid_theta_diff_ = nullptr;
  PidType* pid_roll_ = nullptr;
  PidType* pid_wheel_vel_diff_ = nullptr;

  // Active PID legs populated dynamically during updates
  std::vector<PidType*> active_pid_legs_;

  // State Machine variant
  std::variant<SitDown<PidType, LoggerType, RampFilterType, MovingAverageFilterType>,
               StandUp<PidType, LoggerType, RampFilterType, MovingAverageFilterType>,
               Normal<PidType, LoggerType, RampFilterType, MovingAverageFilterType>,
               Recover<PidType, LoggerType, RampFilterType, MovingAverageFilterType>,
               Upstairs<PidType, LoggerType, RampFilterType, MovingAverageFilterType>,
               Protect<PidType, LoggerType, RampFilterType, MovingAverageFilterType> >
      FSM_variant_;

  void updateEstimation(double dt, const bipedal_wheel_core::SensorMeasurements& sens_in,
                        const bipedal_wheel_core::ControllerCommands& cmd_in);

  LinearKalmanFilter2D kalman_filter_;
  double last_linear_acc_base_x_ = 0.0;
  bool slip_flag_ = false;
};

}  // namespace bipedal_wheel_core

#include "bipedal_wheel/core/bipedal_wheel_core_impl.h"