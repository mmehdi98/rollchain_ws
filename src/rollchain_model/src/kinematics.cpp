#include "rollchain_model/kinematics.hpp"

#include <stdexcept>

namespace rollchain_model
{

Chain forward_kinematics(const std::vector<Joint>& joints, const std::vector<double>& theta)
{
  const std::size_t N = joints.size();
  if (theta.size() != N)
  {
    throw std::invalid_argument("forward_kinematics: theta has " + std::to_string(theta.size()) +
                                " entries, expected " + std::to_string(N));
  }

  Chain c;
  c.top.assign(N + 1, Vec2::Zero());
  c.crest.assign(N, Vec2::Zero());
  c.contact.assign(N, Vec2::Zero());
  c.cg.assign(N, Vec2::Zero());
  c.angle.assign(N + 1, 0.0);

  for (std::size_t k = 0; k < N; ++k)
  {
    const Joint& joint = joints[k];
    const Vec2 a_below = unit(c.angle[k]);
    const Vec2 l_below = rot90(a_below);

    // The contact rolls R theta along the flat top of the link below.
    c.contact[k] = c.top[k] + joint.R * theta[k] * l_below;
    const Vec2 centre = c.contact[k] + joint.R * a_below;

    c.angle[k + 1] = c.angle[k] + theta[k];
    const Vec2 a = unit(c.angle[k + 1]);

    c.crest[k] = centre - joint.R * a;
    c.top[k + 1] = c.crest[k] + joint.L * a;
    c.cg[k] = c.crest[k] + (joint.L - joint.y_cg) * a;
  }
  return c;
}

}  // namespace rollchain_model
