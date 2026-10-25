#pragma once

// The one class the ROS node (or any other user) talks to.
//
// It owns the parameters, the solver settings, the topology and the per-joint
// geometry built from them. Because topology and joints are built in the
// constructor from the parameters, they can never disagree: that is the class
// invariant, and the reason there are no setters. To change a parameter,
// build a new model.

#include <string>
#include <vector>

#include "rollchain_model/coupled.hpp"
#include "rollchain_model/joints.hpp"
#include "rollchain_model/kinematics.hpp"
#include "rollchain_model/loads.hpp"
#include "rollchain_model/naming.hpp"
#include "rollchain_model/outputs.hpp"
#include "rollchain_model/params.hpp"
#include "rollchain_model/statics.hpp"
#include "rollchain_model/topology.hpp"

namespace rollchain_model
{

class RollchainModel
{
public:
  /// Validates the parameters and settings (throws std::invalid_argument
  /// listing every problem), then builds the topology and the joints.
  explicit RollchainModel(RobotParams params, SolverSettings settings = SolverSettings{});

  /// Static equilibrium for the base tensions F (one per rod, rod order).
  /// Negative tensions are treated as 0 (rods cannot push).
  /// kappa: optional sliding sign per rod (theory §6.3); empty = all +1.
  /// Method: settings().method; Automatic = Sequential without loads, Newton with loads.
  /// Throws std::invalid_argument if F or kappa has the wrong size, or if the
  /// sequential method is asked to handle loads.
  State solve(const Tensions& F, const Loads& loads = Loads{}, const std::vector<double>& kappa = {}) const;

  /// Positions along the robot for a solved state.
  Chain kinematics(const State& state) const;

  /// Geometric pull-in of every rod at the base (RC19).
  std::vector<double> pull_in(const State& state) const;

  /// Contact forces of every joint (RC17).
  std::vector<ContactForce> contact_forces(const State& state, const Loads& loads = Loads{},
                                           RodTurning turning = RodTurning::Shared) const;

  /// Strain, stops, contact checks.
  Diagnostics diagnostics(const State& state, const Loads& loads = Loads{}) const;

  const RobotParams& params() const { return params_; }
  const SolverSettings& settings() const { return settings_; }
  const Topology& topology() const { return topology_; }
  const std::vector<Joint>& joints() const { return joints_; }

  std::size_t num_joints() const { return topology_.num_joints(); }
  std::size_t num_rods() const { return topology_.num_rods(); }

  /// Index of the rod called `name`, or num_rods() if there is none.
  std::size_t rod_index(const std::string& name) const;

private:
  // Initialised in this order: joints_ is built from the members above it.
  RobotParams params_;
  SolverSettings settings_;
  Topology topology_;
  std::vector<Joint> joints_;
};

}  // namespace rollchain_model
