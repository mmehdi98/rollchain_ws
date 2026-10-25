#include "rollchain_model/statics.hpp"

#include <algorithm>
#include <cmath>

namespace rollchain_model
{

ValueAndDerivative joint_residual(const Joint& joint, double theta, const Tensions& T_below, double M_load,
                                  const RobotParams& params, const SolverSettings& settings)
{
  const ValueAndDerivative M = elastic_moment(joint, theta, params.lambda0, settings.stop_stiffness);
  const double s = std::sin(theta);
  const double c = std::cos(theta);

  double rod_moment = 0.0;     // sum T (u - R sin theta)
  double total_tension = 0.0;  // sum T
  for (const RodAtJoint& rod : joint.rods)
  {
    const double T = T_below[rod.rod];
    rod_moment += T * (rod.u - joint.R * s);
    total_tension += T;
  }

  return {M.value - rod_moment - M_load, M.derivative + joint.R * c * total_tension};
}

JointSolution solve_joint(const Joint& joint, const Tensions& T_below, double M_load, const RobotParams& params,
                          const SolverSettings& settings)
{
  JointSolution result;
  auto g = [&](double t) { return joint_residual(joint, t, T_below, M_load, params, settings); };

  double lo = -joint.theta_minus - settings.bracket_margin;
  double hi = joint.theta_plus + settings.bracket_margin;
  if (!(g(lo).value < 0.0 && g(hi).value > 0.0))
  {
    // Cannot happen with physical inputs (the stop dominates outside the
    // range); catches NaN or absurd loads instead of returning a wrong angle.
    return result;
  }

  // Start from the linearisation at theta = 0 (RC15).
  const ValueAndDerivative g0 = g(0.0);
  double theta = g0.derivative > 0.0 ? -g0.value / g0.derivative : 0.5 * (lo + hi);
  if (!(theta > lo && theta < hi))
  {
    theta = 0.5 * (lo + hi);
  }

  for (result.iterations = 1; result.iterations <= settings.max_iterations; ++result.iterations)
  {
    const ValueAndDerivative gv = g(theta);
    if (gv.value == 0.0)
    {
      result.converged = true;
      break;
    }

    // g is increasing: the root is below theta if g > 0, above it if g < 0.
    if (gv.value > 0.0)
    {
      hi = theta;
    }
    else
    {
      lo = theta;
    }

    double next = gv.derivative > 0.0 ? theta - gv.value / gv.derivative : 0.5 * (lo + hi);
    if (!(next > lo && next < hi))
    {
      next = 0.5 * (lo + hi);
    }

    const bool small_step = std::abs(next - theta) < settings.angle_tolerance;
    theta = next;
    if (small_step || hi - lo < settings.angle_tolerance)
    {
      result.converged = true;
      break;
    }
  }

  result.theta = theta;
  return result;
}

TensionTable tension_table(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                           const std::vector<double>& theta, const RobotParams& params,
                           const SolverSettings& settings, const std::vector<double>& kappa)
{
  const std::size_t N = joints.size();
  TensionTable table(N + 1);
  Tensions T = F;
  table[0] = T;
  for (std::size_t k = 0; k < N; ++k)
  {
    attenuate(T, joints[k], theta[k], params.mu, kappa, settings.abs_smoothing);
    // Rods that do not cross joint k ended in a link below it.
    for (std::size_t j = 0; j < topology.first_rod_at(k); ++j)
    {
      T[j] = 0.0;
    }
    table[k + 1] = T;
  }
  return table;
}

State solve_sequential(const std::vector<Joint>& joints, const Topology& topology, const Tensions& F,
                       const RobotParams& params, const SolverSettings& settings,
                       const std::vector<double>& M_load, const std::vector<double>& kappa)
{
  const std::size_t N = joints.size();
  const bool loaded = !M_load.empty();

  State state;
  state.method = SolveMethod::Sequential;
  state.theta.assign(N, 0.0);
  state.tension.assign(N + 1, Tensions(F.size(), 0.0));
  state.tension[0] = F;
  state.converged = true;

  // Without loads, joints above the anchor of the last tensioned rod carry
  // no moment at all and stay exactly straight.
  std::size_t last_active = N;
  if (!loaded)
  {
    last_active = 0;
    for (std::size_t j = 0; j < F.size(); ++j)
    {
      if (F[j] > 0.0)
      {
        last_active = std::max(last_active, topology.anchor(j));
      }
    }
  }

  Tensions T = F;
  for (std::size_t k = 0; k < N; ++k)
  {
    if (k < last_active)
    {
      const JointSolution joint = solve_joint(joints[k], T, loaded ? M_load[k] : 0.0, params, settings);
      state.theta[k] = joint.theta;
      state.converged = state.converged && joint.converged;
    }
    attenuate(T, joints[k], state.theta[k], params.mu, kappa, settings.abs_smoothing);
    for (std::size_t j = 0; j < topology.first_rod_at(k); ++j)
    {
      T[j] = 0.0;
    }
    state.tension[k + 1] = T;
  }

  for (std::size_t k = 0; k < N; ++k)
  {
    const double g = joint_residual(joints[k], state.theta[k], state.tension[k], loaded ? M_load[k] : 0.0,
                                    params, settings).value;
    state.residual = std::max(state.residual, std::abs(g));
  }
  return state;
}

}  // namespace rollchain_model
