// Layer 4: mechanics of one joint (theory §6.4, RC9-RC11).

#include <cmath>
#include <functional>

#include <gtest/gtest.h>

#include "fixtures.hpp"

using namespace rollchain_test;

namespace
{

/// Central difference of f at x.
double central(const std::function<double(double)>& f, double x, double h = 1e-6)
{
  return (f(x + h) - f(x - h)) / (2.0 * h);
}

/// lopsy's exact gap heights (C9a/C9b) for a symmetric link (R1 = R2 = R,
/// A1 = A2 = A); returns {left gap, right gap}.
std::pair<double, double> lopsy_gaps(double theta, double A, double R)
{
  const double alpha = std::asin(A / R);
  const double drop = R * (1.0 - std::cos(alpha));
  if (theta >= 0.0)
  {
    const double closing = R * (1.0 - std::cos(alpha - theta));
    const double opening = A * std::sin(theta) + drop * std::cos(theta) + R * (1.0 - std::cos(theta));
    return {closing, opening};  // left closes
  }
  const double a = -theta;
  const double closing = R * (1.0 - std::cos(alpha - a));
  const double opening = A * std::sin(a) + drop * std::cos(a) + R * (1.0 - std::cos(a));
  return {opening, closing};  // right closes
}

class Mechanics : public ::testing::Test
{
protected:
  RobotParams params = reference_robot();
  std::vector<Joint> joints = build_joints(params, Topology(params));
  SolverSettings settings;
};

}  // namespace

TEST(MechanicsFormulas, GapHeightEqualsLopsyForSymmetricLinks)
{
  const double R = 4.0e-3;
  for (double A : {1.0e-3, 2.2e-3, 3.0e-3})
  {
    RodAtJoint left;
    left.u = A;
    left.phi = std::asin(A / R);
    RodAtJoint right;
    right.u = -A;
    right.phi = -std::asin(A / R);
    for (double theta = -0.4; theta <= 0.4; theta += 0.01)
    {
      const auto [hl, hr] = lopsy_gaps(theta, A, R);
      EXPECT_NEAR(gap_height(left, R, theta).value, hl, 1e-15);
      EXPECT_NEAR(gap_height(right, R, theta).value, hr, 1e-15);
    }
  }
}

TEST(MechanicsFormulas, SmoothAbs)
{
  EXPECT_DOUBLE_EQ(smooth_abs(-0.3, 0.0).value, 0.3);
  EXPECT_DOUBLE_EQ(smooth_abs(-0.3, 0.0).derivative, -1.0);
  EXPECT_DOUBLE_EQ(smooth_abs(0.0, 0.0).derivative, 0.0);
  EXPECT_NEAR(smooth_abs(0.2, 1e-4).value, 0.2, 1e-4);
  for (double x : {-0.2, -1e-4, 3e-5, 0.15})
  {
    EXPECT_NEAR(smooth_abs(x, 1e-4).derivative,
                central([](double t) { return smooth_abs(t, 1e-4).value; }, x, 1e-8), 1e-6);
  }
}

TEST(MechanicsFormulas, ArcFactor)
{
  EXPECT_DOUBLE_EQ(arc_factor(0.0).value, 1.0);
  EXPECT_NEAR(arc_factor(0.3).value, 0.3 / std::sin(0.3), 1e-15);
  // The series used below 1e-4 agrees with the exact formula there.
  EXPECT_NEAR(arc_factor(0.999e-4).value, 0.999e-4 / std::sin(0.999e-4), 1e-15);
  for (double t : {-0.3, -0.05, 2e-4, 0.25})
  {
    EXPECT_NEAR(arc_factor(t).derivative, central([](double x) { return arc_factor(x).value; }, t), 1e-8);
  }
}

TEST_F(Mechanics, DerivativesMatchFiniteDifferences)
{
  const double lambda0 = params.lambda0;
  for (const Joint* joint : {&joints[0], &joints[12], &joints[20]})
  {
    for (double theta : {-0.25, -0.1, -0.02, 0.03, 0.12, 0.27})
    {
      for (const RodAtJoint& rod : joint->rods)
      {
        auto h = [&](double t) { return gap_height(rod, joint->R, t).value; };
        EXPECT_NEAR(gap_height(rod, joint->R, theta).derivative, central(h, theta), 1e-10);

        auto s = [&](double t) { return free_length(gap_height(rod, joint->R, t), t, lambda0).value; };
        EXPECT_NEAR(free_length(gap_height(rod, joint->R, theta), theta, lambda0).derivative, central(s, theta),
                    1e-10);
      }
      auto M = [&](double t) { return elastic_moment(*joint, t, lambda0, settings.stop_stiffness).value; };
      EXPECT_NEAR(elastic_moment(*joint, theta, lambda0, settings.stop_stiffness).derivative, central(M, theta),
                  1e-8);
    }
  }
}

TEST_F(Mechanics, ElasticMomentIsOddForASymmetricLayout)
{
  for (double theta : {0.01, 0.1, 0.25})
  {
    const double plus = elastic_moment(joints[0], theta, params.lambda0, settings.stop_stiffness).value;
    const double minus = elastic_moment(joints[0], -theta, params.lambda0, settings.stop_stiffness).value;
    EXPECT_NEAR(plus, -minus, 1e-15);
    EXPECT_GT(plus, 0.0);
  }
  EXPECT_DOUBLE_EQ(elastic_moment(joints[0], 0.0, params.lambda0, settings.stop_stiffness).value, 0.0);
}

TEST_F(Mechanics, ElasticMomentSlopeIsPositiveOverTheRange)
{
  // Theory §7.1: minimum 0.132 N m/rad in the reference robot, so every joint
  // residual is increasing and its root unique.
  double smallest = 1e9;
  for (const Joint* joint : {&joints[0], &joints[12], &joints[20]})
  {
    for (int n = 0; n <= 4000; ++n)
    {
      const double theta = -joint->theta_minus + (joint->theta_plus + joint->theta_minus) * n / 4000.0;
      smallest = std::min(smallest, elastic_moment(*joint, theta, params.lambda0, settings.stop_stiffness).derivative);
    }
  }
  EXPECT_NEAR(smallest, 0.13240097969802933, 1e-12);
}

TEST_F(Mechanics, StopEngagesOnlyOutsideTheRange)
{
  const Joint& j = joints[0];
  EXPECT_DOUBLE_EQ(joint_stop_moment(j, 0.29, settings.stop_stiffness).value, 0.0);
  EXPECT_NEAR(joint_stop_moment(j, 0.31, settings.stop_stiffness).value, settings.stop_stiffness * 0.01, 1e-12);
  EXPECT_NEAR(joint_stop_moment(j, -0.31, settings.stop_stiffness).value, -settings.stop_stiffness * 0.01, 1e-12);
}

TEST_F(Mechanics, RodStrain)
{
  const Joint& j = joints[20];  // section 3: rods at +-2.4 mm
  const double theta = 0.1;
  double expected = 0.0;
  for (const RodAtJoint& rod : j.rods)
  {
    const double s = free_length(gap_height(rod, j.R, theta), theta, params.lambda0).value;
    expected = std::max(expected, rod.r * theta / s);
  }
  EXPECT_DOUBLE_EQ(rod_strain(j, theta, params.lambda0), expected);
  EXPECT_DOUBLE_EQ(rod_strain(j, 0.0, params.lambda0), 0.0);
}
