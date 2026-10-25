#include "rollchain_model/tensions.hpp"

#include <algorithm>
#include <cmath>

#include "rollchain_model/mechanics.hpp"

namespace rollchain_model
{

Tensions clamp_nonnegative(const Tensions& F)
{
  Tensions out(F.size());
  std::transform(F.begin(), F.end(), out.begin(), [](double f) { return std::max(f, 0.0); });
  return out;
}

void attenuate(Tensions& T, const Joint& joint, double theta, double mu, const std::vector<double>& kappa,
               double abs_smoothing)
{
  const double wrap = smooth_abs(theta, abs_smoothing).value;
  for (const RodAtJoint& rod : joint.rods)
  {
    T[rod.rod] *= std::exp(-kappa_of(kappa, rod.rod) * mu * wrap);
  }
}

}  // namespace rollchain_model
