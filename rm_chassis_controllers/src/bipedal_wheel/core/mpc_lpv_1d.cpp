#include "bipedal_wheel/core/mpc_lpv_1d.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <tinympc/tiny_api.hpp>

#include "bipedal_wheel/core/helper_functions.h"

namespace bipedal_wheel_core
{
namespace
{
template <int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> strictlyLower(const Eigen::Matrix<double, Rows, Cols>& matrix)
{
  Eigen::Matrix<double, Rows, Cols> lower = Eigen::Matrix<double, Rows, Cols>::Zero();
  for (int row = 0; row < Rows; ++row)
  {
    for (int col = 0; col <= row && col < Cols; ++col)
      lower(row, col) = matrix(row, col);
  }
  return lower;
}
}  // namespace

bool MpcLpv1d::init(const LqrModelParams& model_params, const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& q,
                    const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& r, const MpcParams& params)
{
  if (!params.enabled)
    return true;
  if (!std::isfinite(params.lpv_leg_min) || !std::isfinite(params.lpv_leg_max) ||
      params.lpv_leg_max <= params.lpv_leg_min || params.lpv_sample_count < 4)
  {
    initialized_ = false;
    return false;
  }

  leg_min_ = params.lpv_leg_min;
  leg_max_ = params.lpv_leg_max;

  std::vector<double> lengths;
  std::vector<Eigen::Matrix<double, STATE_DIM, STATE_DIM>> adyn_samples;
  std::vector<Eigen::Matrix<double, STATE_DIM, CONTROL_DIM>> bdyn_samples;
  std::vector<Eigen::Matrix<double, STATE_DIM, STATE_DIM>> pinf_chol_samples;
  std::vector<Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>> quu_inv_chol_samples;

  lengths.reserve(params.lpv_sample_count);
  adyn_samples.reserve(params.lpv_sample_count);
  bdyn_samples.reserve(params.lpv_sample_count);
  pinf_chol_samples.reserve(params.lpv_sample_count);
  quu_inv_chol_samples.reserve(params.lpv_sample_count);

  for (int i = 0; i < params.lpv_sample_count; ++i)
  {
    const double alpha = params.lpv_sample_count == 1 ? 0.0 : static_cast<double>(i) / (params.lpv_sample_count - 1);
    const double length = leg_min_ + alpha * (leg_max_ - leg_min_);

    ScheduledMpcCache cache;
    if (!computeCacheAtLegLength(model_params, q, r, params, length, cache))
    {
      initialized_ = false;
      return false;
    }

    lengths.push_back(length);
    adyn_samples.push_back(cache.adyn);
    bdyn_samples.push_back(cache.bdyn);
    pinf_chol_samples.push_back(lowerCholeskyPinf(cache.p_inf));
    quu_inv_chol_samples.push_back(lowerCholeskyQuuInv(cache.quu_inv));
  }

  if (!fitPoly(lengths, adyn_samples, adyn_coeffs_) || !fitPoly(lengths, bdyn_samples, bdyn_coeffs_) ||
      !fitPoly(lengths, pinf_chol_samples, pinf_chol_coeffs_) ||
      !fitPoly(lengths, quu_inv_chol_samples, quu_inv_chol_coeffs_))
  {
    initialized_ = false;
    return false;
  }

  initialized_ = true;
  return true;
}

bool MpcLpv1d::evaluate(double leg_length, ScheduledMpcCache& cache) const
{
  if (!initialized_ || !std::isfinite(leg_length))
    return false;

  const double length = std::clamp(leg_length, leg_min_, leg_max_);
  cache.adyn = evalPoly(adyn_coeffs_, length);
  cache.bdyn = evalPoly(bdyn_coeffs_, length);

  Eigen::Matrix<double, STATE_DIM, STATE_DIM> p_chol = strictlyLower(evalPoly(pinf_chol_coeffs_, length));
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> q_chol = strictlyLower(evalPoly(quu_inv_chol_coeffs_, length));

  constexpr double min_diag = 1e-9;
  for (int i = 0; i < STATE_DIM; ++i)
  {
    if (std::abs(p_chol(i, i)) < min_diag)
      p_chol(i, i) = min_diag;
  }
  for (int i = 0; i < CONTROL_DIM; ++i)
  {
    if (std::abs(q_chol(i, i)) < min_diag)
      q_chol(i, i) = min_diag;
  }

  cache.p_inf = p_chol * p_chol.transpose();
  cache.quu_inv = q_chol * q_chol.transpose();
  cache.k_inf = cache.quu_inv * cache.bdyn.transpose() * cache.p_inf * cache.adyn;
  cache.ambk_t = (cache.adyn - cache.bdyn * cache.k_inf).transpose();
  cache.apf.setZero();
  cache.bpf.setZero();

  return cache.adyn.allFinite() && cache.bdyn.allFinite() && cache.p_inf.allFinite() && cache.quu_inv.allFinite() &&
         cache.k_inf.allFinite() && cache.ambk_t.allFinite();
}

template <int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> MpcLpv1d::evalPoly(const PolyMatrix<Rows, Cols>& coeffs, double l)
{
  return ((coeffs[3] * l + coeffs[2]) * l + coeffs[1]) * l + coeffs[0];
}

template <int Rows, int Cols>
bool MpcLpv1d::fitPoly(const std::vector<double>& lengths,
                       const std::vector<Eigen::Matrix<double, Rows, Cols>>& values,
                       PolyMatrix<Rows, Cols>& coeffs)
{
  const int sample_count = static_cast<int>(lengths.size());
  if (sample_count < 4 || values.size() != lengths.size())
    return false;

  Eigen::MatrixXd vandermonde(sample_count, 4);
  Eigen::MatrixXd samples(sample_count, Rows * Cols);
  for (int i = 0; i < sample_count; ++i)
  {
    const double l = lengths[i];
    vandermonde(i, 0) = 1.0;
    vandermonde(i, 1) = l;
    vandermonde(i, 2) = l * l;
    vandermonde(i, 3) = l * l * l;

    Eigen::Map<const Eigen::Matrix<double, Rows * Cols, 1>> flat(values[i].data());
    samples.row(i) = flat.transpose();
  }

  const Eigen::MatrixXd solved = vandermonde.colPivHouseholderQr().solve(samples);
  if (!solved.allFinite())
    return false;

  for (int order = 0; order < 4; ++order)
  {
    Eigen::Map<Eigen::Matrix<double, Rows * Cols, 1>> flat(coeffs[order].data());
    flat = solved.row(order).transpose();
  }
  return true;
}

bool MpcLpv1d::computeCacheAtLegLength(const LqrModelParams& model_params,
                                       const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& q,
                                       const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& r,
                                       const MpcParams& params, double leg_length, ScheduledMpcCache& cache)
{
  if (!std::isfinite(params.dt) || params.dt <= 0.0 || !std::isfinite(params.rho) || params.rho <= 0.0)
    return false;

  Eigen::Matrix<double, STATE_DIM, STATE_DIM> continuous_a{};
  Eigen::Matrix<double, STATE_DIM, CONTROL_DIM> continuous_b{};
  generateAB(model_params, continuous_a, continuous_b, leg_length);

  cache.adyn = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() + params.dt * continuous_a;
  cache.bdyn = params.dt * continuous_b;

  TinyCache tiny_cache;
  const tinyVector fdyn = tinyVector::Zero(STATE_DIM);
  const tinyMatrix q_work = q + params.rho * tinyMatrix::Identity(STATE_DIM, STATE_DIM);
  const tinyMatrix r_work = r + params.rho * tinyMatrix::Identity(CONTROL_DIM, CONTROL_DIM);

  if (tiny_precompute_and_set_cache(&tiny_cache, cache.adyn, cache.bdyn, fdyn, q_work, r_work, STATE_DIM, CONTROL_DIM,
                                    params.rho, 0) != 0)
  {
    return false;
  }

  cache.p_inf = tiny_cache.Pinf;
  cache.quu_inv = tiny_cache.Quu_inv;
  cache.k_inf = tiny_cache.Kinf;
  cache.ambk_t = tiny_cache.AmBKt;
  cache.apf = tiny_cache.APf;
  cache.bpf = tiny_cache.BPf;

  return cache.adyn.allFinite() && cache.bdyn.allFinite() && cache.p_inf.allFinite() && cache.quu_inv.allFinite() &&
         cache.k_inf.allFinite() && cache.ambk_t.allFinite();
}

Eigen::Matrix<double, STATE_DIM, STATE_DIM> MpcLpv1d::lowerCholeskyPinf(
    const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& p_inf)
{
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> matrix = 0.5 * (p_inf + p_inf.transpose());
  Eigen::LLT<Eigen::Matrix<double, STATE_DIM, STATE_DIM>> llt(matrix);
  if (llt.info() != Eigen::Success)
  {
    matrix += 1e-9 * Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    llt.compute(matrix);
  }
  Eigen::Matrix<double, STATE_DIM, STATE_DIM> lower = llt.matrixL();
  return strictlyLower(lower);
}

Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> MpcLpv1d::lowerCholeskyQuuInv(
    const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& quu_inv)
{
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> matrix = 0.5 * (quu_inv + quu_inv.transpose());
  Eigen::LLT<Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>> llt(matrix);
  if (llt.info() != Eigen::Success)
  {
    matrix += 1e-9 * Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>::Identity();
    llt.compute(matrix);
  }
  Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> lower = llt.matrixL();
  return strictlyLower(lower);
}

}  // namespace bipedal_wheel_core
