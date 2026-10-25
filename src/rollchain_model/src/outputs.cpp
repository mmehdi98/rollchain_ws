#include "rollchain_model/outputs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "rollchain_model/loads.hpp"
#include "rollchain_model/mechanics.hpp"

namespace rollchain_model
{

std::vector<double> rod_pull_in(const std::vector<Joint>& joints, const Topology& topology,
                                const std::vector<double>& theta)
{
  std::vector<double> pull(topology.num_rods(), 0.0);
  for (std::size_t k = 0; k < joints.size(); ++k)
  {
    const Joint& joint = joints[k];
    const double lengthening = joint.R * (1.0 - std::cos(theta[k]));
    for (const RodAtJoint& rod : joint.rods)
    {
      pull[rod.rod] += rod.u * theta[k] - lengthening;
    }
  }
  return pull;
}

std::vector<ContactForce> contact_forces(const std::vector<Joint>& joints, const Chain& chain, const State& state,
                                         const Loads& loads, RodTurning turning)
{
  const std::size_t N = joints.size();
  const std::vector<Vec2> W = link_weights(joints, loads);
  const Vec2 F_tip = tip_force(chain, loads);

  std::vector<ContactForce> forces(N);
  Vec2 W_above = Vec2::Zero();
  for (std::size_t k = N; k-- > 0;)
  {
    W_above += W[k];

    double total_tension = 0.0;
    for (const RodAtJoint& rod : joints[k].rods)
    {
      total_tension += state.tension[k][rod.rod];
    }

    double direction = chain.angle[k];  // ByUpperLink: along the lower link
    if (turning == RodTurning::Shared)
    {
      direction += 0.5 * state.theta[k];
    }
    else if (turning == RodTurning::ByLowerLink)
    {
      direction += state.theta[k];
    }

    // Force balance of links k+1..N with the rods inside them (RC17).
    const Vec2 Fc = total_tension * unit(direction) - W_above - F_tip;
    const Vec2 a_below = unit(chain.angle[k]);
    forces[k].normal = Fc.dot(a_below);
    forces[k].tangential = Fc.dot(rot90(a_below));
  }
  return forces;
}

Diagnostics diagnostics(const std::vector<Joint>& joints, const Chain& chain, const State& state,
                        const Loads& loads, const RobotParams& params)
{
  Diagnostics d;
  d.min_normal_force = std::numeric_limits<double>::infinity();

  const std::vector<ContactForce> forces = contact_forces(joints, chain, state, loads);
  for (std::size_t k = 0; k < joints.size(); ++k)
  {
    const double strain = rod_strain(joints[k], state.theta[k], params.lambda0);
    if (strain > d.max_strain)
    {
      d.max_strain = strain;
      d.max_strain_joint = k;
    }
    if (state.theta[k] > joints[k].theta_plus || state.theta[k] < -joints[k].theta_minus)
    {
      d.joints_at_stop.push_back(k);
    }
    d.min_normal_force = std::min(d.min_normal_force, forces[k].normal);
    if (forces[k].normal > 0.0)
    {
      d.max_slip_ratio = std::max(d.max_slip_ratio, std::abs(forces[k].tangential) / forces[k].normal);
    }
  }
  if (joints.empty())
  {
    d.min_normal_force = 0.0;
  }
  return d;
}

}  // namespace rollchain_model
