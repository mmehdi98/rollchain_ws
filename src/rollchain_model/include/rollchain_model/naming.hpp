#pragma once

// Joint names shared with rollchain.urdf.xacro. Kept free of ROS so it can be
// unit-tested; the xacro must build the same names.

#include <cstddef>
#include <string>
#include <vector>

namespace rollchain_model
{

/// "j1_roll", "j2_roll", ..., "j{N}_roll": the revolute joint of each rolling
/// joint. The prismatic j{i}_slide joints are mimics that
/// robot_state_publisher fills in.
std::vector<std::string> urdf_joint_names(std::size_t num_joints);

}  // namespace rollchain_model
