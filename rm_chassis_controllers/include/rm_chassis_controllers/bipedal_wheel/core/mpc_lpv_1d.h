#pragma once

#include <array>
#include <vector>

#include <Eigen/Dense>

#include "bipedal_wheel/core/core_types.h"

namespace bipedal_wheel_core
{

class MpcLpv1d
{
public:
  bool init(const LqrModelParams& model_params, const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& q,
            const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& r, const MpcParams& params);

  bool evaluate(double leg_length, ScheduledMpcCache& cache) const;

  bool initialized() const
  { return initialized_; }

private:
  template <int Rows, int Cols>
  using PolyMatrix = std::array<Eigen::Matrix<double, Rows, Cols>, 4>;

  template <int Rows, int Cols>
  static Eigen::Matrix<double, Rows, Cols> evalPoly(const PolyMatrix<Rows, Cols>& coeffs, double l);

  template <int Rows, int Cols>
  static bool fitPoly(const std::vector<double>& lengths, const std::vector<Eigen::Matrix<double, Rows, Cols>>& values,
                      PolyMatrix<Rows, Cols>& coeffs);

  static bool computeCacheAtLegLength(const LqrModelParams& model_params,
                                      const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& q,
                                      const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& r,
                                      const MpcParams& params, double leg_length, ScheduledMpcCache& cache);

  static Eigen::Matrix<double, STATE_DIM, STATE_DIM> lowerCholeskyPinf(
      const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& p_inf);
  static Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM> lowerCholeskyQuuInv(
      const Eigen::Matrix<double, CONTROL_DIM, CONTROL_DIM>& quu_inv);

  double leg_min_{ 0.12 };
  double leg_max_{ 0.36 };
  bool initialized_{ false };

  PolyMatrix<STATE_DIM, STATE_DIM> adyn_coeffs_{};
  PolyMatrix<STATE_DIM, CONTROL_DIM> bdyn_coeffs_{};
  PolyMatrix<STATE_DIM, STATE_DIM> pinf_chol_coeffs_{};
  PolyMatrix<CONTROL_DIM, CONTROL_DIM> quu_inv_chol_coeffs_{};
};

}  // namespace bipedal_wheel_core
