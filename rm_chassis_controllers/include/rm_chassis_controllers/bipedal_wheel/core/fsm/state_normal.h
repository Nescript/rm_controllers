#pragma once

#include "bipedal_wheel/core/fsm/fsm_context.h"
#include "bipedal_wheel/core/fsm/jump_contact_detector.h"
#include <memory>
#include <Eigen/Dense>

namespace bipedal_wheel_core
{

template <typename PidType, typename LoggerType, typename RampFilterType, typename MovingAverageFilterType>
class Normal
{
public:
  Normal();

  void execute(FsmContext<PidType, LoggerType, RampFilterType, MovingAverageFilterType>& ctx);

private:
  double calculateSupportForce(double F, double Tp, double leg_length, double leg_len_spd, double acc_z,
                               const Eigen::Matrix<double, STATE_DIM, 1>& x, const LqrModelParams& model_params,
                               double dt, bool is_left);

  bool unstickDetection_left(double F_leg, double Tp, double leg_len_spd, double leg_length, double acc_z,
                             const LqrModelParams& model_params, const Eigen::Matrix<double, STATE_DIM, 1>& x,
                             double dt);

  bool unstickDetection_right(double F_leg, double Tp, double leg_len_spd, double leg_length, double acc_z,
                              const LqrModelParams& model_params, const Eigen::Matrix<double, STATE_DIM, 1>& x,
                              double dt);

  std::shared_ptr<MovingAverageFilterType> leftSupportForceAveragePtr_;
  std::shared_ptr<MovingAverageFilterType> rightSupportForceAveragePtr_;

  double pos_des_ = 0.0, leg_length_des = 0.2;
  double unstick_threshold = 15.0;
  JumpPhase jump_phase_ = JumpPhase::IDLE;
  int jumpTime_ = 0;
  bool x_offset_flag_ = false, protect_flag_ = false;
  double jump_cooldown_counter_ = 0.0;
  double off_ground_timer_ = 0.0;
  JumpContactDetector jump_contact_detector_{};

  struct SupportForceHistory
  {
    bool initialized{ false };
    double ddot_zM{ 0.0 }, dot_theta{ 0.0 }, ddot_theta{ 0.0 }, ddot_leg_len{ 0.0 };
    double leg_len_spd[3]{ 0.0, 0.0, 0.0 };
  };
  SupportForceHistory support_force_history_[2];

  bool last_unstick_l_ = false;
  bool last_unstick_r_ = false;
};

}  // namespace bipedal_wheel_core
