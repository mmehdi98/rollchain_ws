// Layer 6: forward kinematics, rod pull-in and joint names (theory §6.2,
// §6.8, §6.9).

#include <cmath>

#include <gtest/gtest.h>

#include "fixtures.hpp"

using namespace rollchain_test;

namespace
{

class Kinematics : public ::testing::Test
{
protected:
  RobotParams params = reference_robot();
  RollchainModel model{params};
  const std::vector<Joint>& joints = model.joints();
};

}  // namespace

TEST_F(Kinematics, StraightRobot)
{
  const Chain c = forward_kinematics(joints, std::vector<double>(24, 0.0));
  EXPECT_NEAR(c.tip()(0), 24 * 15.0e-3, 1e-15);
  EXPECT_DOUBLE_EQ(c.tip()(1), 0.0);
  for (std::size_t k = 0; k < 24; ++k)
  {
    EXPECT_NEAR((c.contact[k] - c.top[k]).norm(), 0.0, 1e-15);
    EXPECT_NEAR(c.cg[k](0), c.top[k](0) + 15.0e-3 - 7.35e-3, 1e-15);
  }
}

TEST_F(Kinematics, OneJointMatchesTheClosedForm)
{
  // (RC18): one bent joint, everything above it straight along the new axis.
  std::vector<double> theta(24, 0.0);
  const double t = 0.2, R = 4.0e-3, L = 15.0e-3;
  theta[0] = t;
  const Chain c = forward_kinematics(joints, theta);
  EXPECT_NEAR(c.top[1](0), R * (1.0 - std::cos(t)) + L * std::cos(t), 1e-15);
  EXPECT_NEAR(c.top[1](1), R * (t - std::sin(t)) + L * std::sin(t), 1e-15);
  EXPECT_NEAR(c.contact[0](1), R * t, 1e-15);
  const Vec2 expected_tip = c.top[1] + 23 * L * unit(t);
  EXPECT_NEAR((c.tip() - expected_tip).norm(), 0.0, 1e-14);
  EXPECT_DOUBLE_EQ(c.tip_angle(), t);
}

TEST_F(Kinematics, LinksRotateAboutTheContactPoint)
{
  // (RC6): dp/dtheta_k = J (p - O_k) for points on links >= k, and the contact
  // point slides along the lower link's top: dO_k/dtheta_k = R l_k.
  const std::vector<double> theta = random_angles(24, 0.25, 7);
  const Chain c = forward_kinematics(joints, theta);
  const double h = 1e-6;  // positions are ~0.3 m: a smaller step drowns in rounding
  for (std::size_t k = 0; k < 24; k += 3)
  {
    std::vector<double> plus = theta, minus = theta;
    plus[k] += h;
    minus[k] -= h;
    const Chain cp = forward_kinematics(joints, plus);
    const Chain cm = forward_kinematics(joints, minus);
    for (std::size_t q = k; q < 24; ++q)
    {
      const Vec2 fd = (cp.cg[q] - cm.cg[q]) / (2 * h);
      EXPECT_NEAR((fd - rot90(c.cg[q] - c.contact[k])).norm(), 0.0, 1e-9);
    }
    const Vec2 fd_tip = (cp.tip() - cm.tip()) / (2 * h);
    EXPECT_NEAR((fd_tip - rot90(c.tip() - c.contact[k])).norm(), 0.0, 1e-9);
    const Vec2 fd_contact = (cp.contact[k] - cm.contact[k]) / (2 * h);
    EXPECT_NEAR((fd_contact - joints[k].R * rot90(unit(c.angle[k]))).norm(), 0.0, 1e-9);
  }
}

TEST_F(Kinematics, MomentOperator)
{
  const Vec2 r(0.3, -0.1), F(0.7, 0.2);
  EXPECT_DOUBLE_EQ(moment(r, F), 0.3 * 0.2 - (-0.1) * 0.7);
  EXPECT_NEAR(moment(rot90(r), F), -r.dot(F), 1e-15);
  // A leftward force above the pivot bends left.
  EXPECT_GT(moment(Vec2(1.0, 0.0), Vec2(0.0, 1.0)), 0.0);
}

TEST_F(Kinematics, PullInIsWorkConsistentWithTheJointBalance)
{
  // (RC19): d(sum_j F_j delta_j)/dtheta_k = sum_j F_j (u_j - R sin theta_k).
  const std::vector<double> theta = random_angles(24, 0.25, 11);
  const Tensions F = F6(3, 1, 4, 1, 5, 9);
  auto work = [&](const std::vector<double>& t) {
    const std::vector<double> d = rod_pull_in(joints, model.topology(), t);
    double w = 0.0;
    for (std::size_t j = 0; j < 6; ++j)
    {
      w += F[j] * d[j];
    }
    return w;
  };
  for (std::size_t k = 0; k < 24; k += 2)
  {
    std::vector<double> plus = theta, minus = theta;
    plus[k] += 1e-6;
    minus[k] -= 1e-6;
    double expected = 0.0;
    for (const RodAtJoint& rod : joints[k].rods)
    {
      expected += F[rod.rod] * (rod.u - joints[k].R * std::sin(theta[k]));
    }
    EXPECT_NEAR((work(plus) - work(minus)) / 2e-6, expected, 1e-9);
  }
}

TEST_F(Kinematics, PullInFirstOrder)
{
  std::vector<double> theta(24, 1e-6);
  const std::vector<double> d = rod_pull_in(joints, model.topology(), theta);
  // The R (1 - cos theta) term is second order: 2e-15 m per joint here.
  EXPECT_NEAR(d[0], 10 * 1.6e-3 * 1e-6, 1e-13);   // 1L crosses 10 joints
  EXPECT_NEAR(d[5], -24 * 2.4e-3 * 1e-6, 1e-13);  // 3R crosses 24 joints
}

TEST(Naming, UrdfJointNames)
{
  const auto names = urdf_joint_names(24);
  ASSERT_EQ(names.size(), 24u);
  EXPECT_EQ(names.front(), "j1_roll");
  EXPECT_EQ(names[9], "j10_roll");
  EXPECT_EQ(names.back(), "j24_roll");
}
