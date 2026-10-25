#pragma once

// Which joints belong to which section, and which rods cross which joint
// (theory §2).
//
// Rods are listed section by section, so the rods crossing joint k,
// J_k = { j : section_j >= sec(k) } (RC1), are always the tail of the rod
// list: [first_rod_at(k), num_rods()). No per-joint lists are needed.

#include <cstddef>
#include <vector>

#include "rollchain_model/params.hpp"

namespace rollchain_model
{

class Topology
{
public:
  /// Expects validated parameters (see validate()).
  explicit Topology(const RobotParams& params);

  std::size_t num_joints() const { return section_of_.size(); }      ///< N
  std::size_t num_sections() const { return last_link_.size() - 1; }  ///< S
  std::size_t num_rods() const { return anchor_.size(); }

  /// N_s: number of links up to and including section s, for s = 0..S (N_0 = 0).
  std::size_t last_link(std::size_t s) const { return last_link_.at(s); }

  /// Section (1-based) of joint k (0-based).
  std::size_t section_of(std::size_t k) const { return section_of_.at(k); }

  /// Index of the first rod of section s (1-based), for s = 1..S+1;
  /// first_rod(S + 1) == num_rods().
  std::size_t first_rod(std::size_t s) const { return first_rod_.at(s); }

  /// The rods crossing joint k are [first_rod_at(k), num_rods()) (RC1).
  std::size_t first_rod_at(std::size_t k) const { return first_rod_.at(section_of_.at(k)); }

  /// Number of joints rod j crosses, i.e. the 1-based index of the link it is
  /// fixed in: N_{s_j}. Rod j crosses joints k = 0..anchor(j)-1.
  std::size_t anchor(std::size_t j) const { return anchor_.at(j); }

  /// True when rod j crosses joint k.
  bool crosses(std::size_t j, std::size_t k) const { return k < anchor_.at(j); }

private:
  std::vector<std::size_t> last_link_;   // size S + 1
  std::vector<std::size_t> section_of_;  // size N
  std::vector<std::size_t> first_rod_;   // size S + 2; index 0 unused
  std::vector<std::size_t> anchor_;      // size num_rods
};

}  // namespace rollchain_model
