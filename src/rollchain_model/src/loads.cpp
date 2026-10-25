#include "rollchain_model/loads.hpp"

#include <cmath>

namespace rollchain_model
{

std::vector<Vec2> link_weights(const std::vector<Joint>& joints, const Loads& loads)
{
  std::vector<Vec2> W(joints.size(), Vec2::Zero());
  if (!loads.gravity_in_plane || loads.scale == 0.0)
  {
    return W;
  }
  const Vec2 g(-std::cos(loads.psi0), std::sin(loads.psi0));
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

std::vector<double> load_moments(const Chain& chain, const std::vector<Joint>& joints, const Loads& loads)
{
  const std::size_t N = joints.size();
  std::vector<double> M(N, 0.0);
  if (!loads.any())
  {
    return M;
  }

  const std::vector<Vec2> W = link_weights(joints, loads);
  const Vec2 F = tip_force(chain, loads);
  const double M_tip = loads.scale * loads.tip_moment;

  // m is bilinear, so sum_{q>=k} m(cg_q - O_k, W_q) = A_k - m(O_k, B_k)
  // with the suffix sums A_k = sum m(cg_q, W_q) and B_k = sum W_q.
  double A = 0.0;
  Vec2 B = Vec2::Zero();
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
  if (!loads.any())
  {
    return arms;
  }

  const std::vector<Vec2> W = link_weights(joints, loads);
  const Vec2 F = tip_force(chain, loads);

  double A = 0.0;          // sum cg_q . W_q
  Vec2 B = Vec2::Zero();   // sum W_q
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
