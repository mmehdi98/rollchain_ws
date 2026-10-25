#include "rollchain_model/coupled.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/LU>

#include "rollchain_model/kinematics.hpp"
#include "rollchain_model/loads.hpp"

namespace rollchain_model
{

namespace
{

double max_abs(const std::vector<double>& v)
{
  double m = 0.0;
  for (double x : v)
  {
    m = std::max(m, std::abs(x));
  }
  return m;
}

}  // namespace

std::vector<double> residual_vector(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                                    const std::vector<double>& theta, const Loads& loads,
                                    const RobotParams& params, const SolverSettings& settings,
                                    const std::vector<double>& kappa)
{
  const std::size_t N = joints.size();
  const TensionTable T = tension_table(joints, topology, F, theta, params, settings, kappa);
  const std::vector<double> M = load_moments(forward_kinematics(joints, theta), joints, loads);

  std::vector<double> g(N);
  for (std::size_t k = 0; k < N; ++k)
  {
    g[k] = joint_residual(joints[k], theta[k], T[k], M[k], params, settings).value;
  }
  return g;
}

Eigen::MatrixXd jacobian(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                         const std::vector<double>& theta, const Loads& loads, const RobotParams& params,
                         const SolverSettings& settings, const std::vector<double>& kappa)
{
  const std::size_t N = joints.size();
  Eigen::MatrixXd Jm = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(N), static_cast<Eigen::Index>(N));

  const TensionTable T = tension_table(joints, topology, F, theta, params, settings, kappa);
  const Chain chain = forward_kinematics(joints, theta);
  const std::vector<Vec2> W = link_weights(joints, loads);
  const Vec2 F_tip = tip_force(chain, loads);
  const LoadArms arms = load_arm_sums(chain, joints, loads);
  const bool dead = !loads.follower;

  // Suffix sums of the weights, for the diagonal term of the moving contact.
  std::vector<Vec2> W_above(N + 1, Vec2::Zero());
  for (std::size_t k = N; k-- > 0;)
  {
    W_above[k] = W_above[k + 1] + W[k];
  }

  // d|theta_k|/dtheta_k
  std::vector<double> dabs(N);
  for (std::size_t k = 0; k < N; ++k)
  {
    dabs[k] = smooth_abs(theta[k], settings.abs_smoothing).derivative;
  }

  for (std::size_t i = 0; i < N; ++i)
  {
    const Joint& joint = joints[i];
    const double s = std::sin(theta[i]);
    const double c = std::cos(theta[i]);
    const auto ii = static_cast<Eigen::Index>(i);

    // Friction: dT_j,i-1/dtheta_k = -mu kappa_j a'_k T_j,i-1 for k < i.
    double friction = 0.0;  // sum_j mu kappa_j T_j (u_j - R sin theta_i)
    double total_tension = 0.0;
    for (const RodAtJoint& rod : joint.rods)
    {
      const double Tj = T[i][rod.rod];
      friction += params.mu * kappa_of(kappa, rod.rod) * Tj * (rod.u - joint.R * s);
      total_tension += Tj;
    }

    const double arm_i = arms.weights[i] + (dead ? arms.tip[i] : 0.0);

    for (std::size_t k = 0; k < i; ++k)
    {
      Jm(ii, static_cast<Eigen::Index>(k)) = dabs[k] * friction + arm_i;
    }

    const Vec2 l_below = rot90(unit(chain.angle[i]));
    const double M_el_slope = elastic_moment(joint, theta[i], params.lambda0, settings.stop_stiffness).derivative;
    Jm(ii, ii) = M_el_slope + joint.R * c * total_tension + arm_i + joint.R * moment(l_below, W_above[i]) +
                 joint.R * moment(l_below, F_tip);

    for (std::size_t k = i + 1; k < N; ++k)
    {
      const double tip_part = dead ? arms.tip[k] : (chain.contact[i] - chain.contact[k]).dot(F_tip);
      Jm(ii, static_cast<Eigen::Index>(k)) = arms.weights[k] + tip_part;
    }
  }
  return Jm;
}

State solve_fixed_point(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                        const Loads& loads, const RobotParams& params, const SolverSettings& settings,
                        const std::vector<double>& kappa)
{
  State state = solve_sequential(joints, topology, F, params, settings, {}, kappa);
  state.method = SolveMethod::FixedPoint;
  if (!loads.any() || !state.converged)
  {
    return state;
  }

  bool settled = false;
  int iterations = 0;
  for (int it = 1; it <= settings.max_coupled_iterations; ++it)
  {
    const std::vector<double> M = load_moments(forward_kinematics(joints, state.theta), joints, loads);
    State next = solve_sequential(joints, topology, F, params, settings, M, kappa);
    double change = 0.0;
    for (std::size_t k = 0; k < joints.size(); ++k)
    {
      change = std::max(change, std::abs(next.theta[k] - state.theta[k]));
    }
    state = std::move(next);
    iterations = it;
    if (!state.converged)
    {
      break;
    }
    if (change < settings.angle_tolerance)
    {
      settled = true;
      break;
    }
  }

  state.method = SolveMethod::FixedPoint;
  state.iterations = iterations;
  state.residual = max_abs(residual_vector(joints, topology, F, state.theta, loads, params, settings, kappa));
  state.converged = state.converged && settled;
  return state;
}

State solve_newton(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                   const Loads& loads, const RobotParams& params, const SolverSettings& settings,
                   const std::vector<double>& kappa)
{
  // Start from the unloaded solution, which the sequential solver finds exactly.
  State state = solve_sequential(joints, topology, F, params, settings, {}, kappa);
  state.method = SolveMethod::Newton;
  if (!loads.any() || !state.converged)
  {
    return state;
  }

  const std::size_t N = joints.size();
  std::vector<double> theta = state.theta;
  double done = 0.0;  // fraction of the load already applied
  double step = 1.0 / settings.load_steps;
  int total_iterations = 0;
  bool ok = true;

  while (done < 1.0)
  {
    const double target = std::min(1.0, done + step);
    Loads partial = loads;
    partial.scale = loads.scale * target;

    std::vector<double> trial_theta = theta;
    bool step_ok = false;
    for (int it = 0; it < settings.max_coupled_iterations; ++it)
    {
      const std::vector<double> g =
          residual_vector(joints, topology, F, trial_theta, partial, params, settings, kappa);
      const double norm = max_abs(g);
      if (norm < settings.residual_tolerance)
      {
        step_ok = true;
        break;
      }

      const Eigen::MatrixXd Jm = jacobian(joints, topology, F, trial_theta, partial, params, settings, kappa);
      const Eigen::Map<const Eigen::VectorXd> gv(g.data(), static_cast<Eigen::Index>(N));
      const Eigen::VectorXd d = Jm.partialPivLu().solve(-gv);
      ++total_iterations;
      if (!d.allFinite())
      {
        break;
      }

      // Backtracking: accept the first step length that reduces the residual.
      bool improved = false;
      double alpha = 1.0;
      for (int b = 0; b < 30 && !improved; ++b, alpha *= 0.5)
      {
        std::vector<double> candidate = trial_theta;
        for (std::size_t k = 0; k < N; ++k)
        {
          candidate[k] += alpha * d(static_cast<Eigen::Index>(k));
        }
        const double candidate_norm =
            max_abs(residual_vector(joints, topology, F, candidate, partial, params, settings, kappa));
        if (candidate_norm < norm)
        {
          trial_theta = std::move(candidate);
          improved = true;
        }
      }
      if (!improved)
      {
        break;
      }
    }

    if (step_ok)
    {
      theta = std::move(trial_theta);
      done = target;
      step = std::min(2.0 * step, 1.0);
    }
    else
    {
      step *= 0.5;
      if (step < 1.0e-6)
      {
        ok = false;
        break;
      }
    }
  }

  state.theta = theta;
  state.tension = tension_table(joints, topology, F, theta, params, settings, kappa);
  state.iterations = total_iterations;
  state.residual = max_abs(residual_vector(joints, topology, F, theta, loads, params, settings, kappa));
  state.converged = ok && state.residual < settings.residual_tolerance;
  return state;
}

}  // namespace rollchain_model
