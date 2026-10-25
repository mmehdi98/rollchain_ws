#include "rollchain_model/params.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>

namespace rollchain_model
{

bool Loads::any() const
{
  if (scale == 0.0)
  {
    return false;
  }
  return gravity_in_plane || tip_force_axial != 0.0 || tip_force_lateral != 0.0 || tip_moment != 0.0;
}

namespace
{

/// Collects messages and throws them all at once.
class Problems
{
public:
  explicit Problems(std::string what) : what_(std::move(what)) {}

  std::ostream& add()
  {
    ++count_;
    out_ << "\n  - ";
    return out_;
  }

  void throw_if_any() const
  {
    if (count_ > 0)
    {
      throw std::invalid_argument("invalid " + what_ + ":" + out_.str());
    }
  }

private:
  std::string what_;
  std::ostringstream out_;
  int count_ = 0;
};

std::string section_label(std::size_t s)
{
  return "section " + std::to_string(s);
}

}  // namespace

void validate(const RobotParams& p)
{
  Problems problems("robot parameters");
  const std::size_t S = p.sections.size();

  if (S == 0)
  {
    problems.add() << "there must be at least one section";
  }
  if (p.rods.empty())
  {
    problems.add() << "there must be at least one rod";
  }
  if (!(p.base_length >= 0.0))
  {
    problems.add() << "base_length must be >= 0";
  }
  if (!(p.mu >= 0.0))
  {
    problems.add() << "mu must be >= 0";
  }
  if (!(p.lambda0 >= 0.0))
  {
    problems.add() << "lambda0 must be >= 0";
  }
  if (!(p.r_h >= 0.0))
  {
    problems.add() << "r_h must be >= 0";
  }
  if (!(p.wall >= 0.0))
  {
    problems.add() << "wall must be >= 0";
  }

  // ---- Sections -----------------------------------------------------------
  for (std::size_t q = 0; q < S; ++q)
  {
    const SectionParams& sec = p.sections[q];
    const std::string label = section_label(q + 1);
    if (sec.n < 1)
    {
      problems.add() << label << ": n must be >= 1";
    }
    if (!(sec.L > 0.0))
    {
      problems.add() << label << ": L must be > 0";
    }
    if (!(sec.R > 0.0))
    {
      problems.add() << label << ": R must be > 0";
    }
    if (!(sec.D > 0.0))
    {
      problems.add() << label << ": D must be > 0";
    }
    if (sec.R > 0.0 && sec.D > 0.0 && sec.R < 0.5 * sec.D)
    {
      problems.add() << label << ": R must be >= D/2 so that the arc spans the link";
    }
    if (!(sec.weight >= 0.0))
    {
      problems.add() << label << ": weight must be >= 0";
    }
    if (!(sec.y_cg >= 0.0 && sec.y_cg <= sec.L))
    {
      problems.add() << label << ": y_cg must lie between 0 and L";
    }
  }

  // ---- Rods ---------------------------------------------------------------
  std::set<std::string> names;
  std::size_t previous_section = 0;
  for (std::size_t j = 0; j < p.rods.size(); ++j)
  {
    const RodParams& rod = p.rods[j];
    const std::string label = "rod '" + rod.name + "'";
    if (rod.name.empty())
    {
      problems.add() << "rod " << j << " has no name";
    }
    else if (!names.insert(rod.name).second)
    {
      problems.add() << label << ": duplicate name";
    }
    if (rod.section < 1 || rod.section > S)
    {
      problems.add() << label << ": section " << rod.section << " does not exist (1.." << S << ")";
      continue;
    }
    if (rod.section < previous_section)
    {
      problems.add() << label << ": rods must be listed section by section (section numbers non-decreasing)";
    }
    previous_section = std::max(previous_section, rod.section);
    if (!(rod.r > 0.0))
    {
      problems.add() << label << ": r must be > 0";
    }
    if (!(rod.E >= 0.0))
    {
      problems.add() << label << ": E must be >= 0";
    }
    const double ax = std::abs(rod.x);
    if (!(ax > p.r_h))
    {
      problems.add() << label << ": |x| must exceed the hole radius r_h, or the joint range (RC5) is empty"
                     << " (a rod on the axis is crossed by the contact line)";
    }
    // Every section the rod crosses: sections 1..rod.section.
    for (std::size_t q = 0; q < rod.section; ++q)
    {
      const SectionParams& sec = p.sections[q];
      if (!(sec.R > 0.0 && sec.D > 0.0 && sec.L > 0.0))
      {
        continue;  // already reported
      }
      if (!(ax < sec.R))
      {
        problems.add() << label << ": |x| must be < R of " << section_label(q + 1)
                       << " so that its hole exit lies on the arc";
        continue;
      }
      if (ax + p.r_h + p.wall > 0.5 * sec.D)
      {
        problems.add() << label << ": hole does not fit inside the link of " << section_label(q + 1)
                       << " (|x| + r_h + wall > D/2)";
      }
      const double hole_length = sec.L - sec.R * (1.0 - std::cos(std::asin(ax / sec.R)));
      if (p.lambda0 > 0.5 * hole_length)
      {
        problems.add() << label << ": lambda0 exceeds half the hole length in " << section_label(q + 1);
      }
    }
  }

  // ---- Every joint needs bending stiffness --------------------------------
  // Joints of section s are crossed by the rods of sections s..S. Without a
  // rod with E > 0 the angle of an unloaded joint is undetermined.
  for (std::size_t q = 0; q < S; ++q)
  {
    const bool stiff = std::any_of(p.rods.begin(), p.rods.end(), [q](const RodParams& rod) {
      return rod.section >= q + 1 && rod.E > 0.0;
    });
    if (!stiff)
    {
      problems.add() << section_label(q + 1) << ": no rod with E > 0 crosses its joints";
    }
  }

  problems.throw_if_any();
}

void validate(const SolverSettings& s)
{
  Problems problems("solver settings");
  if (!(s.abs_smoothing >= 0.0))
  {
    problems.add() << "abs_smoothing must be >= 0";
  }
  if (!(s.bracket_margin > 0.0))
  {
    problems.add() << "bracket_margin must be > 0";
  }
  if (!(s.stop_stiffness > 0.0))
  {
    problems.add() << "stop_stiffness must be > 0";
  }
  if (!(s.angle_tolerance > 0.0))
  {
    problems.add() << "angle_tolerance must be > 0";
  }
  if (s.max_iterations < 1)
  {
    problems.add() << "max_iterations must be >= 1";
  }
  if (!(s.residual_tolerance > 0.0))
  {
    problems.add() << "residual_tolerance must be > 0";
  }
  if (s.max_coupled_iterations < 1)
  {
    problems.add() << "max_coupled_iterations must be >= 1";
  }
  if (s.load_steps < 1)
  {
    problems.add() << "load_steps must be >= 1";
  }
  problems.throw_if_any();
}

std::string to_string(SolveMethod method)
{
  switch (method)
  {
    case SolveMethod::Automatic:
      return "automatic";
    case SolveMethod::Sequential:
      return "sequential";
    case SolveMethod::FixedPoint:
      return "fixed_point";
    case SolveMethod::Newton:
      return "newton";
  }
  return "unknown";
}

SolveMethod solve_method_from_string(const std::string& name)
{
  if (name == "automatic")
  {
    return SolveMethod::Automatic;
  }
  if (name == "sequential")
  {
    return SolveMethod::Sequential;
  }
  if (name == "fixed_point")
  {
    return SolveMethod::FixedPoint;
  }
  if (name == "newton")
  {
    return SolveMethod::Newton;
  }
  throw std::invalid_argument("unknown solve method '" + name +
                              "' (expected automatic, sequential, fixed_point or newton)");
}

}  // namespace rollchain_model
