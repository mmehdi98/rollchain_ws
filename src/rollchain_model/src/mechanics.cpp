#include "rollchain_model/mechanics.hpp"

#include <algorithm>
#include <cmath>

namespace rollchain_model
{

namespace
{
// Guards the free length against 0 when lambda0 = 0 and the contact sits at
// a hole exit. Only reachable outside the joint range, where the stop
// dominates anyway.
constexpr double kMinFreeLength = 1.0e-12;  // [m]
}  // namespace

ValueAndDerivative smooth_abs(double x, double eps)
{
  if (eps <= 0.0)
  {
    return {std::abs(x), x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0)};
  }
  const double q = std::sqrt(x * x + eps * eps);
  return {q - eps, x / q};
}

ValueAndDerivative arc_factor(double theta)
{
  if (std::abs(theta) < 1.0e-4)
  {
    return {1.0 + theta * theta / 6.0, theta / 3.0};
  }
  const double s = std::sin(theta);
  const double c = std::cos(theta);
  return {theta / s, (s - theta * c) / (s * s)};
}

ValueAndDerivative gap_height(const RodAtJoint& rod, double R, double theta)
{
  const double a = rod.phi - theta;
  return {R * (1.0 - std::cos(a)), -R * std::sin(a)};
}

ValueAndDerivative free_length(const ValueAndDerivative& h, double theta, double lambda0)
{
  const ValueAndDerivative rho = arc_factor(theta);
  return {h.value * rho.value + 2.0 * lambda0, h.derivative * rho.value + h.value * rho.derivative};
}

ValueAndDerivative joint_stop_moment(const Joint& joint, double theta, double stop_stiffness)
{
  if (theta > joint.theta_plus)
  {
    return {stop_stiffness * (theta - joint.theta_plus), stop_stiffness};
  }
  if (theta < -joint.theta_minus)
  {
    return {stop_stiffness * (theta + joint.theta_minus), stop_stiffness};
  }
  return {0.0, 0.0};
}

ValueAndDerivative elastic_moment(const Joint& joint, double theta, double lambda0, double stop_stiffness)
{
  double compliance = 0.0;    // sum EI / s
  double d_compliance = 0.0;  // d/dtheta of sum EI / s
  for (const RodAtJoint& rod : joint.rods)
  {
    const ValueAndDerivative s = free_length(gap_height(rod, joint.R, theta), theta, lambda0);
    const double sv = std::max(s.value, kMinFreeLength);
    compliance += rod.EI / sv;
    d_compliance -= rod.EI * s.derivative / (sv * sv);
  }

  ValueAndDerivative M{theta * compliance, compliance + theta * d_compliance};
  const ValueAndDerivative stop = joint_stop_moment(joint, theta, stop_stiffness);
  M.value += stop.value;
  M.derivative += stop.derivative;
  return M;
}

double rod_strain(const Joint& joint, double theta, double lambda0)
{
  double strain = 0.0;
  for (const RodAtJoint& rod : joint.rods)
  {
    const double s = free_length(gap_height(rod, joint.R, theta), theta, lambda0).value;
    strain = std::max(strain, rod.r * std::abs(theta) / std::max(s, kMinFreeLength));
  }
  return strain;
}

}  // namespace rollchain_model
