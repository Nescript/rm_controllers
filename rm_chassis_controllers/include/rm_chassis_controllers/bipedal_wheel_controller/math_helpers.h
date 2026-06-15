#pragma once

#include <cmath>
#include <algorithm>

namespace rm_chassis_controllers
{
template <typename T>
inline void clamp(T& val, const T& minVal, const T& maxVal)
{
  if (val < minVal)
    val = minVal;
  if (val > maxVal)
    val = maxVal;
}

inline double shortest_angular_distance(double from, double to)
{
  double result = fmod(to - from, 2.0 * M_PI);
  if (result > M_PI)
    result -= 2.0 * M_PI;
  else if (result < -M_PI)
    result += 2.0 * M_PI;
  return result;
}

static inline double get_LM(const double& l)
{
  return 0.218 * l + 0.075;
}

static inline double get_theta_leg_offset(const double& l)
{
  return M_PI_4 / 2.0;
}

inline double f_spring_force(double L0, double l1, double l2, double Fs, double s2, double s3, double alpha_s)
{
  double cos_theta3 = (l1 * l1 + l2 * l2 - L0 * L0) / (2 * l1 * l2);
  double theta3 = acos(cos_theta3);
  double ls = sqrt(s2 * s2 + s3 * s3 - 2 * s2 * s3 * cos(theta3 - alpha_s));
  double Fv = Fs * (L0 * s2 * s3 * sin(theta3 - alpha_s)) / (ls * l1 * l2 * sin(theta3));
  return Fv;
}

}  // namespace rm_chassis_controllers
