#pragma once

// Quantities computed on demand from a solved state (theory §6.4, §6.7, §6.9).
// solve() does not compute them.

#include <cstddef>
#include <vector>

#include "rollchain_model/kinematics.hpp"
#include "rollchain_model/statics.hpp"

namespace rollchain_model
{

/// Geometric pull-in of each rod at the base, positive when the rod is drawn
/// toward its actuator (RC19, without the stretch term):
///   delta_j = sum over the joints it crosses of [u theta - R (1 - cos theta)].
std::vector<double> rod_pull_in(const std::vector<Joint>& joints, const Topology& topology,
                                const std::vector<double>& theta);

/// Which hole edges turn the rods at a joint; sets the direction of the rods'
/// force at the cut, and with it the split between normal and tangential
/// contact force (RC17). The angles do not depend on it.
enum class RodTurning
{
  ByUpperLink,  ///< rods leave the lower link straight: force along the lower link's axis (H = 0 without loads)
  Shared,       ///< default: force along the bisector of the two link axes
  ByLowerLink   ///< force along the upper link's axis
};

/// Contact force from link k on link k + 1, in the lower link's frame.
struct ContactForce
{
  double normal = 0.0;      ///< F^R: normal to the lower link's flat top, > 0 in compression [N]
  double tangential = 0.0;  ///< H: along the flat top, + = left [N]
};

/// Contact forces of every joint (RC17).
std::vector<ContactForce> contact_forces(const std::vector<Joint>& joints, const Chain& chain, const State& state,
                                         const Loads& loads, RodTurning turning = RodTurning::Shared);

/// Health checks of a solution (theory §6.4, §6.7).
struct Diagnostics
{
  double max_strain = 0.0;                  ///< peak rod fibre strain
  std::size_t max_strain_joint = 0;         ///< joint index (0-based) of the peak strain
  std::vector<std::size_t> joints_at_stop;  ///< joints outside their range (stop engaged)
  double min_normal_force = 0.0;            ///< smallest F^R [N]; must stay > 0
  double max_slip_ratio = 0.0;              ///< largest |H| / F^R; link-link friction must exceed it
  double stability_margin = 0.0;            ///< see stability_margin() [N m/rad]
  bool stable = true;                       ///< stability_margin > 0: the robot would not buckle away
};

/// The stability check uses the solved tensions with kappa = +1.
Diagnostics diagnostics(const std::vector<Joint>& joints, const Topology& topology, const Chain& chain,
                        const State& state, const Loads& loads, const RobotParams& params,
                        const SolverSettings& settings);

}  // namespace rollchain_model
