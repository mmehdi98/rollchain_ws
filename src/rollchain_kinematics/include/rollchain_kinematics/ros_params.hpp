#pragma once

// Translation between ROS parameters and the rollchain_model structs.
// The only place that knows parameter names; the model library has no ROS.
//
// Parameter names follow the robot config (rollchain_description/config):
//   structure (required, read-only): base.length, sections.n/L/R/D, rods.name/section/x
//   physics (optional, changeable):  sections.weight/y_cg, rods.r/E, holes.lambda0/r_h/wall, friction.mu
//   loads (changeable):              gravity.enabled, gravity.psi0
//   solver (changeable):             solver.*

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rollchain_model/params.hpp"

namespace rollchain_kinematics
{

/// Everything the state publisher reads from its parameters.
struct NodeConfig
{
  rollchain_model::RobotParams robot;
  rollchain_model::SolverSettings solver;
  rollchain_model::Loads loads;
};

/// Declares every robot, solver and gravity parameter on `node` and returns
/// their values. Throws if a structure parameter is missing, or if arrays
/// that must match (per section, per rod) have different lengths.
/// Does not validate the physics: building a RollchainModel does that.
NodeConfig declare_robot_parameters(rclcpp::Node& node);

/// Applies one parameter change to `config`.
/// Returns false, with a reason, for a structure parameter (read-only), a
/// wrong array length or an unknown solver method. Parameters this node does
/// not own (use_sim_time, ...) are accepted and ignored.
bool apply_parameter(NodeConfig& config, const rclcpp::Parameter& parameter, std::string& reason);

}  // namespace rollchain_kinematics
