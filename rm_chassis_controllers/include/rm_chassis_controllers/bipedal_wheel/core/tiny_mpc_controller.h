#pragma once

#include <Eigen/Dense>
#include <tinympc/tiny_api.hpp>

#include "bipedal_wheel/core/core_types.h"

namespace bipedal_wheel_core
{
class TinyMpcController
{
public:
  TinyMpcController() = default;
  ~TinyMpcController();

  TinyMpcController(const TinyMpcController&) = delete;
  TinyMpcController& operator=(const TinyMpcController&) = delete;

  bool init(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& continuous_a,
            const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& continuous_b,
            const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& q,
            const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& r,
            const MpcParams& params);

  bool solve(const Eigen::Matrix<double, STATE_DIM, 1>& state,
             const Eigen::Matrix<double, STATE_DIM, 1>& reference,
             Eigen::Matrix<double, CONTROL_DIM, 1>& control);

  bool updateScheduledCache(const ScheduledMpcCache& cache);

  // Overwrite the horizon input bounds before solving so joint-space torque
  // limits (mapped through the leg Jacobian) can be enforced every cycle.
  bool updateInputBounds(const Eigen::Matrix<double, CONTROL_DIM, 1>& u_min,
                         const Eigen::Matrix<double, CONTROL_DIM, 1>& u_max);

  // Overwrite the horizon velocity-state bounds before solving so rollover
  // constraints can be enforced every cycle.
  bool updateVelStateBounds(double vel_min, double vel_max);

  bool enabled() const { return enabled_; }

private:
  TinySolver* solver_{ nullptr };
  bool enabled_{ false };
};
}  // namespace bipedal_wheel_core