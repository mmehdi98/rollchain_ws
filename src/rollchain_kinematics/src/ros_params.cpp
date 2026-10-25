#include "rollchain_kinematics/ros_params.hpp"

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace rollchain_kinematics
{

namespace rm = rollchain_model;

namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string& text, bool read_only = false)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = text;
  d.read_only = read_only;
  return d;
}

/// A structure parameter: required (no default) and read-only.
template <typename T>
T required(rclcpp::Node& node, const std::string& name, const std::string& text)
{
  return node.declare_parameter<T>(name, describe(text + " (structure: set in the robot config)", true));
}

void expect_size(const std::string& name, std::size_t size, std::size_t expected, const std::string& what)
{
  if (size != expected)
  {
    throw std::invalid_argument(name + " has " + std::to_string(size) + " entries, expected " +
                                std::to_string(expected) + " (one per " + what + ")");
  }
}

/// Optional per-item array: empty means "use the default for every item".
void apply_optional(const std::string& name, const std::vector<double>& values, std::size_t count,
                    const std::string& what)
{
  if (!values.empty())
  {
    expect_size(name, values.size(), count, what);
  }
}

void set_section_weights(rm::RobotParams& robot, const std::vector<double>& weight)
{
  for (std::size_t q = 0; q < robot.sections.size(); ++q)
  {
    robot.sections[q].weight = weight.empty() ? 0.0 : weight[q];
  }
}

void set_section_cg(rm::RobotParams& robot, const std::vector<double>& y_cg)
{
  for (std::size_t q = 0; q < robot.sections.size(); ++q)
  {
    robot.sections[q].y_cg = y_cg.empty() ? 0.5 * robot.sections[q].L : y_cg[q];
  }
}

void set_rod_radius(rm::RobotParams& robot, const std::vector<double>& r)
{
  for (std::size_t j = 0; j < robot.rods.size(); ++j)
  {
    robot.rods[j].r = r.empty() ? rm::RodParams{}.r : r[j];
  }
}

void set_rod_modulus(rm::RobotParams& robot, const std::vector<double>& E)
{
  for (std::size_t j = 0; j < robot.rods.size(); ++j)
  {
    robot.rods[j].E = E.empty() ? rm::RodParams{}.E : E[j];
  }
}

}  // namespace

NodeConfig declare_robot_parameters(rclcpp::Node& node)
{
  NodeConfig config;
  rm::RobotParams& robot = config.robot;

  // ---- Structure: required, read-only --------------------------------------
  robot.base_length = required<double>(node, "base.length", "base height below its flat top [m]");

  const auto n = required<std::vector<int64_t>>(node, "sections.n", "links per section");
  const auto L = required<std::vector<double>>(node, "sections.L", "link length per section, crest to flat top [m]");
  const auto R = required<std::vector<double>>(node, "sections.R", "bottom-arc radius per section [m]");
  const auto D = required<std::vector<double>>(node, "sections.D", "link diameter per section [m]");
  const std::size_t S = n.size();
  expect_size("sections.L", L.size(), S, "section");
  expect_size("sections.R", R.size(), S, "section");
  expect_size("sections.D", D.size(), S, "section");
  robot.sections.resize(S);
  for (std::size_t q = 0; q < S; ++q)
  {
    if (n[q] < 1)
    {
      throw std::invalid_argument("sections.n must be >= 1 for every section");
    }
    robot.sections[q].n = static_cast<std::size_t>(n[q]);
    robot.sections[q].L = L[q];
    robot.sections[q].R = R[q];
    robot.sections[q].D = D[q];
  }

  const auto names = required<std::vector<std::string>>(node, "rods.name", "rod names, listed section by section");
  const auto section = required<std::vector<int64_t>>(node, "rods.section", "section of each rod (from 1)");
  const auto x = required<std::vector<double>>(node, "rods.x", "lateral offset of each rod [m], + = left");
  // Depth along the joint axis: structure, but optional (empty = all 0).
  const auto w = node.declare_parameter<std::vector<double>>(
      "rods.w", std::vector<double>{},
      describe("depth of each rod along the joint axis [m]; empty = 0 (structure: set in the robot config)", true));
  const std::size_t n_rods = names.size();
  expect_size("rods.section", section.size(), n_rods, "rod");
  expect_size("rods.x", x.size(), n_rods, "rod");
  apply_optional("rods.w", w, n_rods, "rod");
  robot.rods.resize(n_rods);
  for (std::size_t j = 0; j < n_rods; ++j)
  {
    if (section[j] < 1)
    {
      throw std::invalid_argument("rods.section must be >= 1 for every rod");
    }
    robot.rods[j].name = names[j];
    robot.rods[j].section = static_cast<std::size_t>(section[j]);
    robot.rods[j].x = x[j];
    robot.rods[j].w = w.empty() ? 0.0 : w[j];
  }

  // ---- Physics: optional, changeable at runtime ----------------------------
  const auto weight = node.declare_parameter<std::vector<double>>(
      "sections.weight", std::vector<double>{}, describe("weight of one link per section [N]; empty = 0"));
  const auto y_cg = node.declare_parameter<std::vector<double>>(
      "sections.y_cg", std::vector<double>{}, describe("CG depth below the flat top per section [m]; empty = L/2"));
  apply_optional("sections.weight", weight, S, "section");
  apply_optional("sections.y_cg", y_cg, S, "section");
  set_section_weights(robot, weight);
  set_section_cg(robot, y_cg);

  const auto r = node.declare_parameter<std::vector<double>>(
      "rods.r", std::vector<double>{}, describe("rod radius per rod [m]; empty = 0.3 mm"));
  const auto E = node.declare_parameter<std::vector<double>>(
      "rods.E", std::vector<double>{}, describe("equivalent bending modulus per rod [Pa]; empty = 61 GPa"));
  apply_optional("rods.r", r, n_rods, "rod");
  apply_optional("rods.E", E, n_rods, "rod");
  set_rod_radius(robot, r);
  set_rod_modulus(robot, E);

  const rm::RobotParams d;
  robot.lambda0 = node.declare_parameter("holes.lambda0", d.lambda0, describe("setback inside each hole (C17) [m]"));
  robot.r_h = node.declare_parameter("holes.r_h", d.r_h, describe("hole radius [m]; sets the joint range"));
  robot.wall = node.declare_parameter("holes.wall", d.wall, describe("minimum wall to the link edge [m]"));
  robot.mu = node.declare_parameter("friction.mu", d.mu, describe("effective rod-link friction coefficient"));

  // ---- Loads ------------------------------------------------------------------
  config.loads.gravity_in_plane =
      node.declare_parameter("gravity.enabled", false, describe("gravity acts in the bending plane"));
  config.loads.psi0 = node.declare_parameter(
      "gravity.psi0", 0.0, describe("base axis from the upward vertical [rad], + toward the left; 0 upright, pi hanging"));

  // ---- Solver -----------------------------------------------------------------
  const rm::SolverSettings s;
  rm::SolverSettings& solver = config.solver;
  solver.method = rm::solve_method_from_string(node.declare_parameter(
      "solver.method", rm::to_string(s.method), describe("automatic, sequential, fixed_point or newton")));
  solver.abs_smoothing =
      node.declare_parameter("solver.abs_smoothing", s.abs_smoothing, describe("smoothing of |theta| [rad]; 0 = exact"));
  solver.bracket_margin = node.declare_parameter("solver.bracket_margin", s.bracket_margin,
                                                 describe("joint root bracket beyond the range [rad]"));
  solver.stop_stiffness = node.declare_parameter("solver.stop_stiffness", s.stop_stiffness,
                                                 describe("joint stop stiffness [N m/rad]"));
  solver.angle_tolerance =
      node.declare_parameter("solver.angle_tolerance", s.angle_tolerance, describe("angle tolerance [rad]"));
  solver.max_iterations = static_cast<int>(node.declare_parameter<int64_t>(
      "solver.max_iterations", s.max_iterations, describe("iterations per joint")));
  solver.residual_tolerance = node.declare_parameter("solver.residual_tolerance", s.residual_tolerance,
                                                     describe("coupled solvers: residual tolerance [N m]"));
  solver.max_coupled_iterations = static_cast<int>(node.declare_parameter<int64_t>(
      "solver.max_coupled_iterations", s.max_coupled_iterations, describe("coupled solvers: iteration cap")));
  solver.load_steps = static_cast<int>(
      node.declare_parameter<int64_t>("solver.load_steps", s.load_steps, describe("Newton continuation steps")));

  return config;
}

bool apply_parameter(NodeConfig& config, const rclcpp::Parameter& p, std::string& reason)
{
  const std::string& name = p.get_name();
  rm::RobotParams& robot = config.robot;
  try
  {
    if (name == "base.length" || name == "sections.n" || name == "sections.L" ||
        name == "sections.R" || name == "sections.D" || name == "rods.name" || name == "rods.section" ||
        name == "rods.x" || name == "rods.w")
    {
      reason = name + " is a structure parameter: change the robot config and restart";
      return false;
    }
    if (name == "sections.weight")
    {
      apply_optional(name, p.as_double_array(), robot.sections.size(), "section");
      set_section_weights(robot, p.as_double_array());
    }
    else if (name == "sections.y_cg")
    {
      apply_optional(name, p.as_double_array(), robot.sections.size(), "section");
      set_section_cg(robot, p.as_double_array());
    }
    else if (name == "rods.r")
    {
      apply_optional(name, p.as_double_array(), robot.rods.size(), "rod");
      set_rod_radius(robot, p.as_double_array());
    }
    else if (name == "rods.E")
    {
      apply_optional(name, p.as_double_array(), robot.rods.size(), "rod");
      set_rod_modulus(robot, p.as_double_array());
    }
    else if (name == "holes.lambda0")
    {
      robot.lambda0 = p.as_double();
    }
    else if (name == "holes.r_h")
    {
      robot.r_h = p.as_double();
    }
    else if (name == "holes.wall")
    {
      robot.wall = p.as_double();
    }
    else if (name == "friction.mu")
    {
      robot.mu = p.as_double();
    }
    else if (name == "gravity.enabled")
    {
      config.loads.gravity_in_plane = p.as_bool();
    }
    else if (name == "gravity.psi0")
    {
      config.loads.psi0 = p.as_double();
    }
    else if (name == "solver.method")
    {
      config.solver.method = rm::solve_method_from_string(p.as_string());
    }
    else if (name == "solver.abs_smoothing")
    {
      config.solver.abs_smoothing = p.as_double();
    }
    else if (name == "solver.bracket_margin")
    {
      config.solver.bracket_margin = p.as_double();
    }
    else if (name == "solver.stop_stiffness")
    {
      config.solver.stop_stiffness = p.as_double();
    }
    else if (name == "solver.angle_tolerance")
    {
      config.solver.angle_tolerance = p.as_double();
    }
    else if (name == "solver.max_iterations")
    {
      config.solver.max_iterations = static_cast<int>(p.as_int());
    }
    else if (name == "solver.residual_tolerance")
    {
      config.solver.residual_tolerance = p.as_double();
    }
    else if (name == "solver.max_coupled_iterations")
    {
      config.solver.max_coupled_iterations = static_cast<int>(p.as_int());
    }
    else if (name == "solver.load_steps")
    {
      config.solver.load_steps = static_cast<int>(p.as_int());
    }
    // Anything else is not ours: accept it unchanged.
  }
  catch (const std::exception& e)
  {
    reason = name + ": " + e.what();
    return false;
  }
  return true;
}

}  // namespace rollchain_kinematics
