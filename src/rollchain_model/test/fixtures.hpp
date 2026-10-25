#pragma once

// Robots and helpers shared by the tests.

#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "rollchain_model/model.hpp"

namespace rollchain_test
{

using namespace rollchain_model;

/// The reference robot of theory §11.2 (also config/reference_robot.yaml):
/// S = 3, n = (10, 8, 6), L = 15 mm, R = 4 mm, rods at x = +-(1.6, 2.0, 2.4) mm
/// and depths w = (-1, +1, 0) mm so that the 0.8 mm holes do not overlap.
/// Rod order: 1L 1R 2L 2R 3L 3R.
inline RobotParams reference_robot()
{
  RobotParams p;
  p.base_length = 15.0e-3;
  for (std::size_t n : {10u, 8u, 6u})
  {
    SectionParams s;
    s.n = n;
    s.L = 15.0e-3;
    s.R = 4.0e-3;
    s.D = 6.2e-3;
    s.weight = 5.685e-3;
    s.y_cg = 7.35e-3;
    p.sections.push_back(s);
  }
  const double a[3] = {1.6e-3, 2.0e-3, 2.4e-3};
  const double depth[3] = {-1.0e-3, 1.0e-3, 0.0};
  for (std::size_t s = 1; s <= 3; ++s)
  {
    for (int side : {+1, -1})
    {
      RodParams rod;
      rod.name = std::to_string(s) + (side > 0 ? "L" : "R");
      rod.section = s;
      rod.x = side * a[s - 1];
      rod.w = depth[s - 1];
      rod.r = 0.3e-3;
      rod.E = 61.0e9;
      p.rods.push_back(rod);
    }
  }
  p.mu = 0.66;
  p.lambda0 = 2.2e-3;
  p.r_h = 0.4e-3;
  p.wall = 0.2e-3;
  return p;
}

/// One section of 15 links with rods at +-2.2 mm: the symmetric case of
/// lopsy (A1 = A2 = 2.2 mm, R1 = R2 = 4 mm). Rod order: L, R.
inline RobotParams one_section_robot()
{
  RobotParams p = reference_robot();
  p.sections.resize(1);
  p.sections[0].n = 15;
  p.rods.clear();
  for (int side : {+1, -1})
  {
    RodParams rod;
    rod.name = side > 0 ? "L" : "R";
    rod.section = 1;
    rod.x = side * 2.2e-3;
    p.rods.push_back(rod);
  }
  return p;
}

/// Base tensions of the reference robot, in rod order.
inline Tensions F6(double f1L, double f1R, double f2L, double f2R, double f3L, double f3R)
{
  return {f1L, f1R, f2L, f2R, f3L, f3R};
}

/// Settings that force a given method.
inline SolverSettings with_method(SolveMethod method)
{
  SolverSettings s;
  s.method = method;
  return s;
}

/// Reproducible random angles in [-amplitude, amplitude].
inline std::vector<double> random_angles(std::size_t n, double amplitude, unsigned seed)
{
  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> dist(-amplitude, amplitude);
  std::vector<double> theta(n);
  for (double& t : theta)
  {
    t = dist(gen);
  }
  return theta;
}

}  // namespace rollchain_test
