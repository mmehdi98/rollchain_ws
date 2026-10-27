#include "rollchain_model/loads.hpp"

#include <cmath>

namespace rollchain_model
{

namespace
{

/// Unit vector of gravity in the base frame: (-cos psi0, sin psi0).
Vec2 gravity_direction(const Loads& loads)
{
  return Vec2(-std::cos(loads.psi0), std::sin(loads.psi0));
}

}  // namespace

std::vector<Vec2> link_weights(const std::vector<Joint>& joints, const Loads& loads)
{
  std::vector<Vec2> W(joints.size(), Vec2::Zero());
  if (!loads.gravity_in_plane || loads.scale == 0.0)
  {
    return W;
  }
  const Vec2 g = gravity_direction(loads);
  for (std::size_t k = 0; k < joints.size(); ++k)
  {
    W[k] = loads.scale * joints[k].weight * g;
  }
  return W;
}

Vec2 tip_force(const Chain& chain, const Loads& loads)
{
  const Vec2 local = loads.scale * Vec2(loads.tip_force_axial, loads.tip_force_lateral);
  if (!loads.follower)
  {
    return local;
  }
  const Vec2 a = unit(chain.tip_angle());
  return local(0) * a + local(1) * rot90(a);
}

Vec2 payload_force(const Loads& loads)
{
  if (!loads.gravity_in_plane)
  {
    return Vec2::Zero();
  }
  return loads.scale * loads.payload_weight * gravity_direction(loads);
}

std::vector<double> load_moments(const Chain& chain, const std::vector<Joint>& joints, const Loads& loads)
{
  const std::size_t N = joints.size();
  std::vector<double> M(N, 0.0);
  if (!loads.any() || N == 0)
  {
    return M;
  }

  const std::vector<Vec2> W = link_weights(joints, loads);
  const Vec2 F = tip_force(chain, loads);
  const double M_tip = loads.scale * loads.tip_moment;

  // m is bilinear, so sum_{q>=k} m(p_q - O_k, W_q) = A_k - m(O_k, B_k)
  // with the suffix sums A_k = sum m(p_q, W_q) and B_k = sum W_q. The payload
  // is a weight at the tip, carried by every joint.
  const Vec2 Wp = payload_force(loads);
  double A = moment(chain.tip(), Wp);
  Vec2 B = Wp;
  for (std::size_t k = N; k-- > 0;)
  {
    A += moment(chain.cg[k], W[k]);
    B += W[k];
    M[k] = A - moment(chain.contact[k], B) + moment(chain.tip() - chain.contact[k], F) + M_tip;
  }
  return M;
}

LoadArms load_arm_sums(const Chain& chain, const std::vector<Joint>& joints, const Loads& loads)
{
  const std::size_t N = joints.size();
  LoadArms arms{std::vector<double>(N, 0.0), std::vector<double>(N, 0.0)};
  if (!loads.any() || N == 0)
  {
    return arms;
  }

  const std::vector<Vec2> W = link_weights(joints, loads);
  const Vec2 F = tip_force(chain, loads);
  const Vec2 Wp = payload_force(loads);

  double A = chain.tip().dot(Wp);  // sum p_q . W_q
  Vec2 B = Wp;                      // sum W_q
  for (std::size_t k = N; k-- > 0;)
  {
    A += chain.cg[k].dot(W[k]);
    B += W[k];
    arms.weights[k] = A - chain.contact[k].dot(B);
    arms.tip[k] = (chain.tip() - chain.contact[k]).dot(F);
  }
  return arms;
}

}  // namespace rollchain_model
