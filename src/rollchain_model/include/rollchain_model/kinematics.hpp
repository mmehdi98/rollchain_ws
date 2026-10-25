#pragma once

// Planar forward kinematics in the base frame (theory §3.3, §6.2, §6.8).
//
// Vectors are (axial, lateral), lateral positive to the left. The base frame
// has its origin at the top centre of the base, X axial and Y lateral.
//
// URDF mapping (rollchain_description): URDF z = axial, URDF x = lateral
// (left), joints rotate about +y, and base_link sits base_length below this
// frame's origin.

#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "rollchain_model/joints.hpp"

namespace rollchain_model
{

/// (axial, lateral) [m] or [N].
using Vec2 = Eigen::Vector2d;

/// Bending moment operator (RC2): m(r, F) = r_a F_l - r_l F_a.
/// Positive promotes positive theta (bending left).
inline double moment(const Vec2& r, const Vec2& F)
{
  return r(0) * F(1) - r(1) * F(0);
}

/// J: rotates a vector by +90 deg, from axial toward lateral.
/// Identity used by the Jacobian: moment(rot90(r), F) == -r.dot(F).
inline Vec2 rot90(const Vec2& v)
{
  return Vec2(-v(1), v(0));
}

/// Unit vector at absolute angle `angle` from the base axis: (cos, sin).
inline Vec2 unit(double angle)
{
  return Vec2(std::cos(angle), std::sin(angle));
}

/// Positions along the robot for one set of joint angles.
struct Chain
{
  std::vector<Vec2> top;      ///< N + 1: top centre of each link; top[0] is the base origin
  std::vector<Vec2> crest;    ///< N: crest of link k + 1 (index k)
  std::vector<Vec2> contact;  ///< N: contact point O of joint k (theory §6.2)
  std::vector<Vec2> cg;       ///< N: centre of gravity of link k + 1
  std::vector<double> angle;  ///< N + 1: absolute axis angle Theta of each link; angle[0] = 0

  const Vec2& tip() const { return top.back(); }
  double tip_angle() const { return angle.back(); }
};

/// Builds the chain joint by joint (RC18). For joint k, with a and l the
/// axial and lateral unit vectors of link k:
///   contact = top[k] + R theta_k l,   arc centre = contact + R a,
///   crest   = centre - R a',          top[k+1] = crest + L a',
/// where a' is the axial unit vector of link k + 1.
Chain forward_kinematics(const std::vector<Joint>& joints, const std::vector<double>& theta);

}  // namespace rollchain_model
