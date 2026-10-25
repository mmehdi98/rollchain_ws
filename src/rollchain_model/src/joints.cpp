#include "rollchain_model/joints.hpp"

#include <algorithm>
#include <cmath>

namespace rollchain_model
{

std::vector<Joint> build_joints(const RobotParams& params, const Topology& topology)
{
  const std::size_t N = topology.num_joints();
  std::vector<Joint> joints(N);

  for (std::size_t k = 0; k < N; ++k)
  {
    Joint& joint = joints[k];
    const std::size_t s = topology.section_of(k);
    const SectionParams& sec = params.sections[s - 1];

    joint.index = k;
    joint.section = s;
    joint.R = sec.R;
    joint.L = sec.L;
    joint.D = sec.D;
    joint.weight = sec.weight;
    joint.y_cg = sec.y_cg;

    // Range: the contact must not reach a hole, nor the edge of the link.
    const double edge = 0.5 * sec.D / sec.R;
    joint.theta_plus = edge;
    joint.theta_minus = edge;
    joint.k0 = 0.0;

    for (std::size_t j = topology.first_rod_at(k); j < topology.num_rods(); ++j)
    {
      const RodParams& rp = params.rods[j];
      RodAtJoint rod;
      rod.rod = j;
      rod.u = rp.x;  // planar robot: the lever offset is the lateral offset
      rod.phi = std::asin(rod.u / sec.R);
      rod.h0 = sec.R * (1.0 - std::cos(rod.phi));
      rod.EI = rp.E * kPi * std::pow(rp.r, 4) / 4.0;
      rod.r = rp.r;
      joint.rods.push_back(rod);

      const double reach = (std::abs(rod.u) - params.r_h) / sec.R;  // (RC5)
      if (rod.u > 0.0)
      {
        joint.theta_plus = std::min(joint.theta_plus, reach);
      }
      else if (rod.u < 0.0)
      {
        joint.theta_minus = std::min(joint.theta_minus, reach);
      }

      joint.k0 += rod.EI / (rod.h0 + 2.0 * params.lambda0);  // (RC4)
    }
  }
  return joints;
}

}  // namespace rollchain_model
