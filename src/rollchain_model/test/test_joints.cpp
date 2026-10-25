// Layer 3: per-joint geometry (theory §5, RC3-RC5).

#include <cmath>

#include <gtest/gtest.h>

#include "fixtures.hpp"

using namespace rollchain_test;

TEST(Joints, RodsAtEachJoint)
{
  const RobotParams p = reference_robot();
  const Topology t(p);
  const std::vector<Joint> joints = build_joints(p, t);
  ASSERT_EQ(joints.size(), 24u);

  EXPECT_EQ(joints[0].rods.size(), 6u);
  EXPECT_EQ(joints[10].rods.size(), 4u);
  EXPECT_EQ(joints[23].rods.size(), 2u);
  EXPECT_EQ(joints[10].rods.front().rod, 2u);  // 2L
  EXPECT_EQ(joints[23].section, 3u);

  const RodAtJoint& r = joints[0].rods[4];  // 3L at joint 1
  EXPECT_EQ(r.rod, 4u);
  EXPECT_DOUBLE_EQ(r.u, 2.4e-3);
  EXPECT_DOUBLE_EQ(r.phi, std::asin(2.4e-3 / 4.0e-3));
  EXPECT_DOUBLE_EQ(r.h0, 4.0e-3 * (1.0 - std::cos(r.phi)));
  EXPECT_DOUBLE_EQ(r.EI, 61.0e9 * kPi * std::pow(0.3e-3, 4) / 4.0);
}

TEST(Joints, RangeIsSetByTheInnermostHole)
{
  // (RC5): (a_min - r_h) / R = 0.3, 0.4, 0.5 rad in sections 1, 2, 3
  // (17.19, 22.92, 28.65 deg, theory §11.2).
  const RobotParams p = reference_robot();
  const std::vector<Joint> joints = build_joints(p, Topology(p));
  EXPECT_NEAR(joints[0].theta_plus, 0.3, 1e-12);
  EXPECT_NEAR(joints[12].theta_plus, 0.4, 1e-12);
  EXPECT_NEAR(joints[20].theta_plus, 0.5, 1e-12);
  for (const Joint& j : joints)
  {
    EXPECT_DOUBLE_EQ(j.theta_minus, j.theta_plus);  // symmetric layout
  }
}

TEST(Joints, RangeFallsBackToTheLinkEdge)
{
  // Rods on the left only: nothing limits the right side but the edge, D/(2R).
  RobotParams p = reference_robot();
  p.rods = {p.rods[0], p.rods[2], p.rods[4]};  // 1L, 2L, 3L
  ASSERT_NO_THROW(validate(p));
  const std::vector<Joint> joints = build_joints(p, Topology(p));
  EXPECT_DOUBLE_EQ(joints[0].theta_minus, 0.5 * 6.2e-3 / 4.0e-3);
  EXPECT_NEAR(joints[0].theta_plus, 0.3, 1e-12);
}

TEST(Joints, StiffnessAtZero)
{
  // (RC4), theory §11.2: 0.4704, 0.3065, 0.1493 N m/rad.
  const RobotParams p = reference_robot();
  const std::vector<Joint> joints = build_joints(p, Topology(p));
  EXPECT_NEAR(joints[0].k0, 0.4704480756683023, 1e-14);
  EXPECT_NEAR(joints[15].k0, 0.30649784473595115, 1e-14);
  EXPECT_NEAR(joints[23].k0, 0.14925585866718427, 1e-14);
}

TEST(Joints, OneSectionRobot)
{
  const RobotParams p = one_section_robot();
  const std::vector<Joint> joints = build_joints(p, Topology(p));
  ASSERT_EQ(joints.size(), 15u);
  EXPECT_NEAR(joints[0].theta_plus, 0.45, 1e-12);  // 25.78 deg
  EXPECT_NEAR(joints[0].k0, 0.15340543490192804, 1e-14);
}
