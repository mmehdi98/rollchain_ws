#include "rollchain_model/topology.hpp"

namespace rollchain_model
{

Topology::Topology(const RobotParams& params)
{
  const std::size_t S = params.sections.size();

  last_link_.assign(S + 1, 0);
  for (std::size_t s = 1; s <= S; ++s)
  {
    last_link_[s] = last_link_[s - 1] + params.sections[s - 1].n;
  }

  section_of_.assign(last_link_[S], 0);
  for (std::size_t s = 1; s <= S; ++s)
  {
    for (std::size_t k = last_link_[s - 1]; k < last_link_[s]; ++k)
    {
      section_of_[k] = s;
    }
  }

  // first_rod_[s]: first rod whose section is >= s. Rods are sorted by section.
  const std::size_t n_rods = params.rods.size();
  first_rod_.assign(S + 2, n_rods);
  first_rod_[0] = 0;
  for (std::size_t s = 1; s <= S + 1; ++s)
  {
    std::size_t j = 0;
    while (j < n_rods && params.rods[j].section < s)
    {
      ++j;
    }
    first_rod_[s] = j;
  }

  anchor_.resize(n_rods);
  for (std::size_t j = 0; j < n_rods; ++j)
  {
    anchor_[j] = last_link_.at(params.rods[j].section);
  }
}

}  // namespace rollchain_model
