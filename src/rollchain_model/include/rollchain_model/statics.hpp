#pragma once

// Joint balance and the joint-by-joint solver (theory §6.5, §7.1).
//
// Joint k depends on the joints below it only through the rod tensions
// reaching it, and its residual is increasing in its own angle, so without
// loads the N joints are solved one at a time from the base, each with a
// unique root.

#include <vector>

#include "rollchain_model/mechanics.hpp"
#include "rollchain_model/params.hpp"
#include "rollchain_model/tensions.hpp"
#include "rollchain_model/topology.hpp"

namespace rollchain_model
{

/// Result of one static solve. Vectors are indexed from the base.
struct State
{
  std::vector<double> theta;  ///< joint angles theta_1..theta_N [rad]; > 0 = bent left
  TensionTable tension;       ///< N + 1 rows; see TensionTable
  bool converged = false;     ///< every joint (and the coupled iteration) converged
  int iterations = 0;         ///< coupled iterations (fixed point or Newton); 0 for sequential
  SolveMethod method = SolveMethod::Sequential;  ///< the solver that produced this state
  double residual = 0.0;      ///< largest |g_k| at the solution [N m]
};

/// Residual of the joint balance (RC20) and its slope:
///   g  = M_el(theta) - sum_j T_j (u_j - R sin theta) - M_load
///   g' = M_el'(theta) + R cos theta sum_j T_j
/// T_below: tensions just below the joint (one per rod, rod order); only the
/// rods crossing the joint are read. M_load: known load moment (RC16), 0
/// without loads.
ValueAndDerivative joint_residual(const Joint& joint, double theta, const Tensions& T_below, double M_load,
                                  const RobotParams& params, const SolverSettings& settings);

/// Outcome of the scalar root search at one joint.
struct JointSolution
{
  double theta = 0.0;
  int iterations = 0;
  bool converged = false;
};

/// Root of joint_residual() in (-theta_minus - margin, theta_plus + margin):
/// bracketed Newton started from the linearisation at 0 (RC15), with a
/// bisection step whenever Newton would leave the bracket.
JointSolution solve_joint(const Joint& joint, const Tensions& T_below, double M_load, const RobotParams& params,
                          const SolverSettings& settings);

/// Tension table for given angles (RC7): row 0 = F, row k+1 = row k
/// attenuated across joint k; a rod's entries are 0 above its anchor link.
TensionTable tension_table(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                           const std::vector<double>& theta, const RobotParams& params,
                           const SolverSettings& settings, const std::vector<double>& kappa = {});

/// Joint-by-joint solve from the base (theory §7.1).
/// F: base tensions, one per rod, already non-negative.
/// M_load: known load moment per joint (empty = no loads). With no loads,
/// joints above the anchor of the last tensioned rod are exactly straight
/// and are not solved.
State solve_sequential(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                       const RobotParams& params, const SolverSettings& settings,
                       const std::vector<double>& M_load = {}, const std::vector<double>& kappa = {});

}  // namespace rollchain_model
