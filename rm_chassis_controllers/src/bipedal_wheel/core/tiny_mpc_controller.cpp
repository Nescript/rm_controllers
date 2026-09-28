#include "bipedal_wheel/core/tiny_mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace bipedal_wheel_core
{
TinyMpcController::~TinyMpcController()
{
  if (solver_ != nullptr)
  {
    delete solver_->solution;
    delete solver_->settings;
    delete solver_->cache;
    delete solver_->work;
    delete solver_;
  }
}

bool TinyMpcController::init(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& continuous_a,
                             const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& continuous_b,
                             const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& q,
                             const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& r,
                             const MpcParams& params)
{
  if (!params.enabled)
    return true;

  enabled_ = true;
  const int horizon = std::max(2, params.horizon);
  const int max_iterations = std::max(1, params.max_iter);
  const double dt = params.dt;
  const double rho = params.rho;
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(rho) || rho <= 0.0)
  {
    enabled_ = false;
    return false;
  }

  const Eigen::Matrix<double, STATE_DIM, STATE_DIM> discrete_a =
      Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() + dt * continuous_a;
  const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> discrete_b = dt * continuous_b;
  const tinyMatrix a = discrete_a;
  const tinyMatrix b = discrete_b;
  const tinyVector f = tinyVector::Zero(STATE_DIM);
  const tinyMatrix tiny_q = q;
  const tinyMatrix tiny_r = r;

  if (tiny_setup(&solver_, a, b, f, tiny_q, tiny_r, rho, STATE_DIM, CONTROL_DIM, horizon, 0) != 0)
  {
    enabled_ = false;
    return false;
  }

  tinyMatrix x_min = tinyMatrix::Constant(STATE_DIM, horizon, -1e17);
  tinyMatrix x_max = tinyMatrix::Constant(STATE_DIM, horizon, 1e17);
  tinyMatrix u_min = tinyMatrix::Zero(CONTROL_DIM, horizon - 1);
  tinyMatrix u_max = tinyMatrix::Zero(CONTROL_DIM, horizon - 1);
  for (int step = 0; step < horizon - 1; ++step)
  {
    for (int input = 0; input < CONTROL_DIM; ++input)
    {
      // The second YAML value is a joint motor limit, not a virtual leg torque.
      // Normal updates this input using the current VMC Jacobian before solving.
      u_min(input, step) = input == LEG_Tp ? -1e17 : params.input_min[input];
      u_max(input, step) = input == LEG_Tp ? 1e17 : params.input_max[input];
    }
  }
  if (tiny_set_bound_constraints(solver_, x_min, x_max, u_min, u_max) != 0)
  {
    enabled_ = false;
    return false;
  }
  solver_->settings->max_iter = max_iterations;
  solver_->settings->check_termination = 1;

  return true;
}

bool TinyMpcController::solve(const Eigen::Matrix<double, STATE_DIM, 1>& state,
                              const Eigen::Matrix<double, STATE_DIM, 1>& reference,
                              Eigen::Matrix<double, CONTROL_DIM, 1>& control)
{
  if (!enabled_ || solver_ == nullptr || !state.allFinite() || !reference.allFinite())
    return false;

  const tinyVector x0 = state;
  if (tiny_set_x0(solver_, x0) != 0)
    return false;
  solver_->work->Xref = tinyMatrix(reference).replicate(1, solver_->work->N);
  solver_->work->Uref.setZero();
  if (tiny_solve(solver_) != 0 || solver_->solution->solved == 0)
    return false;

  const auto first_control = solver_->work->u.col(0);
  if (!first_control.allFinite())
    return false;
  control = first_control;
  return true;
}

bool TinyMpcController::updateScheduledCache(const ScheduledMpcCache& cache)
{
  if (!enabled_ || solver_ == nullptr || !cache.adyn.allFinite() || !cache.bdyn.allFinite() ||
      !cache.k_inf.allFinite() || !cache.p_inf.allFinite() || !cache.quu_inv.allFinite() ||
      !cache.ambk_t.allFinite() || !cache.apf.allFinite() || !cache.bpf.allFinite())
  {
    return false;
  }

  solver_->work->Adyn = cache.adyn;
  solver_->work->Bdyn = cache.bdyn;
  solver_->work->fdyn.setZero();

  solver_->cache->Kinf = cache.k_inf;
  solver_->cache->Pinf = cache.p_inf;
  solver_->cache->Quu_inv = cache.quu_inv;
  solver_->cache->AmBKt = cache.ambk_t;
  solver_->cache->APf = cache.apf;
  solver_->cache->BPf = cache.bpf;
  solver_->cache->C1 = cache.quu_inv;
  solver_->cache->C2 = cache.ambk_t;
  return true;
}

bool TinyMpcController::updateInputBounds(const Eigen::Matrix<double, CONTROL_DIM, 1>& u_min,
                                          const Eigen::Matrix<double, CONTROL_DIM, 1>& u_max)
{
  if (!enabled_ || solver_ == nullptr || !u_min.allFinite() || !u_max.allFinite())
    return false;
  for (int step = 0; step < solver_->work->N - 1; ++step)
  {
    solver_->work->u_min.col(step) = u_min;
    solver_->work->u_max.col(step) = u_max;
  }
  return true;
}

bool TinyMpcController::updateVelStateBounds(double vel_min, double vel_max)
{
  if (!enabled_ || solver_ == nullptr || !std::isfinite(vel_min) || !std::isfinite(vel_max))
    return false;
  for (int step = 0; step < solver_->work->N; ++step)
  {
    solver_->work->x_min(VEL, step) = vel_min;
    solver_->work->x_max(VEL, step) = vel_max;
  }
  return true;
}
}  // namespace bipedal_wheel_core
