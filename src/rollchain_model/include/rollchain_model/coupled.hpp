#pragma once

// Solvers for the coupled problem with gravity or tip loads (theory §7.2, §8).
// The load moments make each joint depend on every joint above it, so the
// joint-by-joint structure is lost.

#include <vector>

#include <Eigen/Core>

#include "rollchain_model/joints.hpp"
#include "rollchain_model/statics.hpp"

namespace rollchain_model
{

/// g_k = M_el_k - sum_j T_j,k-1 (u_j - R_k sin theta_k) - M^G_k - M^ext_k for every joint (RC12).
std::vector<double> residual_vector(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                                    const std::vector<double>& theta, const Loads& loads,
                                    const RobotParams& params, const SolverSettings& settings,
                                    const std::vector<double>& kappa = {});

/// Analytic Jacobian dg_i/dtheta_k (RC22), for dead and follower tip loads.
Eigen::MatrixXd jacobian(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                         const std::vector<double>& theta, const Loads& loads, const RobotParams& params,
                         const SolverSettings& settings, const std::vector<double>& kappa = {});

/// Fixed point: load moments from the previous shape, then a sequential
/// solve, repeated until the angles stop changing (theory §7.2).
State solve_fixed_point(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                        const Loads& loads, const RobotParams& params, const SolverSettings& settings,
                        const std::vector<double>& kappa = {});

/// Newton on (RC12) with the analytic Jacobian, a backtracking line search,
/// and continuation in the load scale from 0 to 1 (theory §7.2).
State solve_newton(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                   const Loads& loads, const RobotParams& params, const SolverSettings& settings,
                   const std::vector<double>& kappa = {});

}  // namespace rollchain_model
