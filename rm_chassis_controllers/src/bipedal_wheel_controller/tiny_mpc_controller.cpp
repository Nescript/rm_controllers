#include "bipedal_wheel_controller/tiny_mpc_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rm_chassis_controllers
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

bool TinyMpcController::init(const ros::NodeHandle& node_handle,
                             const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& continuous_a,
                             const Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>& continuous_b,
                             const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& q,
                             const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& r)
{
  if (!node_handle.param("mpc_enabled", enabled_, false) || !enabled_)
    return true;

  const int horizon = std::max(2, node_handle.param("mpc_horizon", 15));
  const int max_iterations = std::max(1, node_handle.param("mpc_max_iter", 50));
  const double dt = node_handle.param("mpc_dt", 0.01);
  const double rho = node_handle.param("mpc_rho", 1.0);
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(rho) || rho <= 0.0)
  {
    ROS_ERROR("Invalid TinyMPC dt or rho");
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
    ROS_ERROR("Failed to initialize TinyMPC");
    enabled_ = false;
    return false;
  }

  std::vector<double> input_min;
  std::vector<double> input_max;
  if (!node_handle.getParam("mpc_input_min", input_min) || !node_handle.getParam("mpc_input_max", input_max) ||
      input_min.size() != CONTROL_DIM || input_max.size() != CONTROL_DIM)
  {
    ROS_ERROR("mpc_input_min and mpc_input_max must contain two values");
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
      u_min(input, step) = input_min[input];
      u_max(input, step) = input_max[input];
    }
  }
  if (tiny_set_bound_constraints(solver_, x_min, x_max, u_min, u_max) != 0)
  {
    ROS_ERROR("Failed to configure TinyMPC input bounds");
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
}  // namespace rm_chassis_controllers