#pragma once

// Rod tensions and the capstan law (theory §6.3).

#include <vector>

#include "rollchain_model/joints.hpp"

namespace rollchain_model
{

/// One tension per rod, in rod order [N].
using Tensions = std::vector<double>;

/// tension[k][j]: tension of rod j just above link k [N].
/// Row 0 holds the base tensions, row k + 1 the tensions after joint k.
/// A rod's entries are 0 above the link it is fixed in.
using TensionTable = std::vector<Tensions>;

/// Rods cannot push: negative entries become 0.
Tensions clamp_nonnegative(const Tensions& F);

/// Applies the capstan factor of joint `joint` to the rods crossing it (RC7):
///   T_j <- T_j exp(-kappa_j mu |theta|).
/// kappa holds one sliding sign per rod (+1: the rod slides toward its
/// actuator, so its tension falls toward the tip); empty means all +1.
/// Entries of rods that do not cross the joint are left unchanged.
void attenuate(Tensions& T, const Joint& joint, double theta, double mu, const std::vector<double>& kappa,
               double abs_smoothing);

/// Sliding sign of rod j: kappa[j], or +1 when kappa is empty.
inline double kappa_of(const std::vector<double>& kappa, std::size_t j)
{
  return kappa.empty() ? 1.0 : kappa[j];
}

}  // namespace rollchain_model
