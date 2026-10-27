#pragma once

// Input data of the model: the robot (structure and physics), the loads of one
// solve and the solver settings. Plain structs; validate() checks them.
//
// Units: SI everywhere (m, N, rad, Pa).
// Indexing: sections are numbered from 1, as in the config file and the
// theory. Joints and links are stored 0-based: element k is joint/link
// i = k + 1 of the theory.

#include <cstddef>
#include <string>
#include <vector>

namespace rollchain_model
{

constexpr double kPi = 3.14159265358979323846;
constexpr double deg2rad(double deg) { return deg * kPi / 180.0; }
constexpr double rad2deg(double rad) { return rad * 180.0 / kPi; }

/// One section of identical symmetric links (theory §1, §4.1).
struct SectionParams
{
  std::size_t n = 1;      ///< number of links (= joints) in the section
  double L = 15.0e-3;     ///< link length, crest to flat top [m]
  double R = 4.0e-3;      ///< bottom-arc radius [m]
  double D = 6.2e-3;      ///< link diameter [m]
  double weight = 0.0;    ///< weight of one link [N]; used only with in-plane gravity
  double y_cg = 7.35e-3;  ///< CG depth below the flat top [m]; x_cg = 0 by symmetry
};

/// One rod (theory §2.2, §4.2).
struct RodParams
{
  std::string name;         ///< unique label, e.g. "1L"
  std::size_t section = 1;  ///< section the rod belongs to (1-based); fixed in its last link
  double x = 0.0;           ///< signed lateral offset from the link axis [m], + = left
  double w = 0.0;           ///< depth along the joint axis [m]; places the hole (fit, overlap,
                            ///< drawing) but does not enter the planar statics (theory §9.2)
  double r = 0.3e-3;        ///< rod radius [m]
  double E = 61.0e9;        ///< equivalent bending modulus [Pa]; 0 = a cable with no bending stiffness
};

/// The whole robot.
struct RobotParams
{
  double base_length = 15.0e-3;         ///< base height below its flat top [m]; drawing only
  std::vector<SectionParams> sections;  ///< base to tip
  std::vector<RodParams> rods;          ///< listed section by section
  double mu = 0.66;                     ///< effective rod-link friction for the |theta| wrap
  double lambda0 = 2.2e-3;              ///< setback: depth a rod is clamped inside each hole (C17) [m]
  double r_h = 0.4e-3;                  ///< hole radius [m]; sets the joint range (RC5)
  double wall = 0.2e-3;                 ///< minimum wall between a hole and the link edge [m]; validation only
};

/// Loads of one solve (theory §6.6). Not part of the robot: the same robot
/// can lie on a table, stand upright or hang.
struct Loads
{
  bool gravity_in_plane = false;   ///< gravity acts in the bending plane
  double psi0 = 0.0;               ///< base axis from the upward vertical, + toward the left [rad]; 0 upright, pi hanging
  double tip_force_axial = 0.0;    ///< tip force, axial component [N]
  double tip_force_lateral = 0.0;  ///< tip force, lateral component, + = left [N]
  double tip_moment = 0.0;         ///< tip moment, + promotes bending left [N m]
  bool follower = false;           ///< false: the force is fixed in the base frame; true: it turns with the tip
  double payload_weight = 0.0;     ///< weight of a mass carried at the tip [N]; acts along gravity, so only
                                   ///< when gravity_in_plane is true
  double scale = 1.0;              ///< multiplies every load; the Newton solver uses it for continuation

  /// True when any load acts, i.e. when the joints are coupled (theory §7.2).
  bool any() const;
};

enum class SolveMethod
{
  Automatic,   ///< Sequential without loads, Newton with loads
  Sequential,  ///< joint by joint from the base (theory §7.1); no loads
  FixedPoint,  ///< loads from the previous shape, then a sequential solve, repeated (theory §7.2)
  Newton       ///< all joints together with the analytic Jacobian (theory §7.2, §8)
};

/// Numerical settings. Kept apart from RobotParams because they are not
/// properties of the robot.
struct SolverSettings
{
  double abs_smoothing = 0.0;          ///< smoothing scale of |theta| in the capstan law [rad]; 0 = exact
  double bracket_margin = 0.2;         ///< joint root bracket (-theta_minus - m, theta_plus + m) [rad]
  double stop_stiffness = 10.0;        ///< one-sided joint stop past the range [N m/rad]
  double angle_tolerance = 1.0e-12;    ///< per-joint and fixed-point convergence [rad]
  int max_iterations = 100;            ///< per joint
  SolveMethod method = SolveMethod::Automatic;
  double residual_tolerance = 1.0e-12; ///< coupled solvers: largest |g_i| [N m]
  int max_coupled_iterations = 200;    ///< Newton iterations per load step, or fixed-point iterations
  int load_steps = 10;                 ///< Newton continuation steps from no load to the full load
};

/// Throws std::invalid_argument listing every problem found.
void validate(const RobotParams& params);
void validate(const SolverSettings& settings);

/// "automatic", "sequential", "fixed_point", "newton".
std::string to_string(SolveMethod method);
/// Inverse of to_string(); throws std::invalid_argument for an unknown name.
SolveMethod solve_method_from_string(const std::string& name);

}  // namespace rollchain_model
