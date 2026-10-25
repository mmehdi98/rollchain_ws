#pragma once

// Mechanics of ONE joint: pure functions of that joint's angle theta
// (theory §6.4). No loops over joints here; the statics layer does that.
// Every function returns the value and its derivative d/dtheta together.

#include "rollchain_model/joints.hpp"

namespace rollchain_model
{

/// A scalar and its derivative with respect to theta.
struct ValueAndDerivative
{
  double value = 0.0;
  double derivative = 0.0;
};

/// |x| for the capstan wrap (theory §6.3).
/// eps = 0: exact |x| with derivative sign(x), taken as 0 at x = 0.
/// eps > 0: sqrt(x^2 + eps^2) - eps, smooth through 0.
ValueAndDerivative smooth_abs(double x, double eps);

/// rho(theta) = theta / sin(theta): turns a gap height into the length of a
/// rod bent into a circular arc. Series near 0: 1 + theta^2/6, rho' = theta/3.
ValueAndDerivative arc_factor(double theta);

/// Height of rod j's hole exit above the flat top of the link below, after
/// the joint rolls by theta (RC9): h = R (1 - cos(phi - theta)).
/// One formula for every rod and both signs of theta.
ValueAndDerivative gap_height(const RodAtJoint& rod, double R, double theta);

/// Free length of a rod across the joint (RC10):
///   s = h rho(theta) + 2 lambda0,   s' = h' rho + h rho'.
ValueAndDerivative free_length(const ValueAndDerivative& h, double theta, double lambda0);

/// One-sided joint stop past the range (theory §5.3):
/// k_stop (theta - theta_plus) above it, k_stop (theta + theta_minus) below it.
ValueAndDerivative joint_stop_moment(const Joint& joint, double theta, double stop_stiffness);

/// Elastic moment of all rods crossing the joint, positive opposing +theta
/// (RC10)-(RC11):  M = theta sum_j EI_j / s_j,  M' = sum_j EI_j (1/s_j - theta s_j'/s_j^2),
/// plus the joint stop.
ValueAndDerivative elastic_moment(const Joint& joint, double theta, double lambda0, double stop_stiffness);

/// Largest rod fibre strain at the joint, max_j r_j |theta| / s_j (theory §6.4).
double rod_strain(const Joint& joint, double theta, double lambda0);

}  // namespace rollchain_model
