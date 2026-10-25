#pragma once

// Per-joint geometry, computed once from the parameters (theory §5).
// Replaces lopsy's per-link Link struct.
//
// In the planar robot every joint of a section is identical apart from its
// index. They are stored per joint anyway: it costs nothing, and the spatial
// version (theory §10) needs it, because there the lever offset u of a rod
// changes from joint to joint.

#include <cstddef>
#include <vector>

#include "rollchain_model/params.hpp"
#include "rollchain_model/topology.hpp"

namespace rollchain_model
{

/// One rod as seen by one joint.
struct RodAtJoint
{
  std::size_t rod = 0;  ///< index into RobotParams::rods and into a Tensions vector
  double u = 0.0;       ///< lever offset u_ij [m]; = x_j in the planar robot (theory §5.1)
  double phi = 0.0;     ///< hole-exit angle on the arc, asin(u / R) (RC3) [rad]
  double h0 = 0.0;      ///< gap height when straight, R (1 - cos phi) (RC3) [m]
  double EI = 0.0;      ///< bending stiffness E pi r^4 / 4 [N m^2]
  double r = 0.0;       ///< rod radius [m]
};

/// One rolling joint: link k + 1 rolling on the flat top of link k.
struct Joint
{
  std::size_t index = 0;       ///< 0-based joint index k (theory i = k + 1)
  std::size_t section = 1;     ///< section of the joint (1-based)
  double R = 0.0;              ///< arc radius of the rolling link [m]
  double L = 0.0;              ///< length of the rolling link, crest to flat top [m]
  double D = 0.0;              ///< diameter of the rolling link [m]
  std::vector<RodAtJoint> rods;  ///< the rods crossing this joint (RC1), in rod order
  double theta_plus = 0.0;     ///< joint range toward the left (RC5) [rad]
  double theta_minus = 0.0;    ///< joint range toward the right (RC5), positive [rad]
  double k0 = 0.0;             ///< joint stiffness at theta = 0 (RC4) [N m/rad]
  double weight = 0.0;         ///< weight of the rolling link [N]
  double y_cg = 0.0;           ///< CG depth of the rolling link below its flat top [m]
};

/// Builds the N joints, base to tip. Expects validated parameters.
///
/// The range (RC5) is set by the innermost hole on each side, and never
/// exceeds the edge of the link: theta_plus = min(D/(2R), min over rods with
/// u > 0 of (u - r_h)/R), and likewise theta_minus for u < 0.
std::vector<Joint> build_joints(const RobotParams& params, const Topology& topology);

}  // namespace rollchain_model
