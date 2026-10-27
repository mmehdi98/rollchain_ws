#include "rollchain_model/model.hpp"

#include <stdexcept>
#include <utility>

namespace rollchain_model
{

namespace
{

RobotParams checked(RobotParams params)
{
  validate(params);
  return params;
}

SolverSettings checked(const SolverSettings& settings)
{
  validate(settings);
  return settings;
}

}  // namespace

RollchainModel::RollchainModel(RobotParams params, SolverSettings settings)
  : params_(checked(std::move(params))),
    settings_(checked(settings)),
    topology_(params_),
    joints_(build_joints(params_, topology_))
{
}

State RollchainModel::solve(const Tensions& F, const Loads& loads, const std::vector<double>& kappa) const
{
  if (F.size() != num_rods())
  {
    throw std::invalid_argument("solve: got " + std::to_string(F.size()) + " tensions for " +
                                std::to_string(num_rods()) + " rods");
  }
  if (!(loads.payload_weight >= 0.0))
  {
    throw std::invalid_argument("solve: payload_weight must be >= 0");
  }
  if (!kappa.empty() && kappa.size() != num_rods())
  {
    throw std::invalid_argument("solve: kappa must be empty or have one entry per rod");
  }
  const Tensions F0 = clamp_nonnegative(F);

  SolveMethod method = settings_.method;
  if (method == SolveMethod::Automatic)
  {
    method = loads.any() ? SolveMethod::Newton : SolveMethod::Sequential;
  }

  switch (method)
  {
    case SolveMethod::Sequential:
      if (loads.any())
      {
        throw std::invalid_argument("solve: the sequential method cannot handle gravity or tip loads; "
                                    "use automatic, fixed_point or newton");
      }
      return solve_sequential(joints_, topology_, F0, params_, settings_, {}, kappa);
    case SolveMethod::FixedPoint:
      return solve_fixed_point(joints_, topology_, F0, loads, params_, settings_, kappa);
    case SolveMethod::Newton:
    case SolveMethod::Automatic:
      return solve_newton(joints_, topology_, F0, loads, params_, settings_, kappa);
  }
  return {};
}

Chain RollchainModel::kinematics(const State& state) const
{
  return forward_kinematics(joints_, state.theta);
}

std::vector<double> RollchainModel::pull_in(const State& state) const
{
  return rod_pull_in(joints_, topology_, state.theta);
}

std::vector<ContactForce> RollchainModel::contact_forces(const State& state, const Loads& loads,
                                                         RodTurning turning) const
{
  return rollchain_model::contact_forces(joints_, kinematics(state), state, loads, turning);
}

Diagnostics RollchainModel::diagnostics(const State& state, const Loads& loads) const
{
  return rollchain_model::diagnostics(joints_, topology_, kinematics(state), state, loads, params_, settings_);
}

std::size_t RollchainModel::rod_index(const std::string& name) const
{
  for (std::size_t j = 0; j < params_.rods.size(); ++j)
  {
    if (params_.rods[j].name == name)
    {
      return j;
    }
  }
  return num_rods();
}

}  // namespace rollchain_model
