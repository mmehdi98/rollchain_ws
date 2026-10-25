#pragma once

// Gravity and tip loads in the joint balance (theory §6.6, §8).

#include <vector>

#include "rollchain_model/kinematics.hpp"

namespace rollchain_model
{

/// Weight vector of each link in the base frame (zero without in-plane
/// gravity): W_k = scale * weight_k * (-cos psi0, sin psi0).
std::vector<Vec2> link_weights(const std::vector<Joint>& joints, const Loads& loads);

/// Tip force in the base frame, scaled. A follower force is given in the tip
/// frame and turned by the tip angle.
Vec2 tip_force(const Chain& chain, const Loads& loads);

/// M^G_k + M^ext_k for every joint (RC16): the moment about the contact point
/// O_k of the weights of links k+1..N and of the tip load.
std::vector<double> load_moments(const Chain& chain, const std::vector<Joint>& joints, const Loads& loads);

/// Load-arm sums for the Jacobian (RC21), split by load:
///   weights[k] = sum_{q >= k} (cg_q - O_k) . W_q
///   tip[k]     = (tip - O_k) . F_tip
struct LoadArms
{
  std::vector<double> weights;
  std::vector<double> tip;
};
LoadArms load_arm_sums(const Chain& chain, const std::vector<Joint>& joints, const Loads& loads);

}  // namespace rollchain_model
