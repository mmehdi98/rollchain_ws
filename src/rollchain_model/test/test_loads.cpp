// Layer 7: gravity, tip loads, the Jacobian and the coupled solvers
// (theory §6.6, §6.7, §7.2, §8, §11.2 cases E and F).

#include <cmath>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "fixtures.hpp"

using namespace rollchain_test;

namespace
{

class Loaded : public ::testing::Test
{
protected:
  RobotParams params = reference_robot();
  RollchainModel model{params};
  const std::vector<Joint>& joints = model.joints();
  const Topology& topology = model.topology();
};

Loads gravity(double psi0)
{
  Loads l;
  l.gravity_in_plane = true;
  l.psi0 = psi0;
  return l;
}

}  // namespace

TEST_F(Loaded, LoadMomentIsTheDerivativeOfTheLoadsWork)
{
  for (double psi0 : {0.0, kPi, 0.7})
  {
    Loads loads = gravity(psi0);
    loads.tip_force_axial = 0.3;
    loads.tip_force_lateral = -0.2;
    loads.tip_moment = 0.01;
    const std::vector<double> theta = random_angles(24, 0.25, 3);
    const std::vector<double> M = load_moments(forward_kinematics(joints, theta), joints, loads);

    auto work = [&](const std::vector<double>& t) {
      const Chain c = forward_kinematics(joints, t);
      const std::vector<Vec2> W = link_weights(joints, loads);
      double w = tip_force(c, loads).dot(c.tip()) + loads.tip_moment * c.tip_angle();
      for (std::size_t q = 0; q < 24; ++q)
      {
        w += W[q].dot(c.cg[q]);
      }
      return w;
    };
    for (std::size_t k = 0; k < 24; ++k)
    {
      std::vector<double> plus = theta, minus = theta;
      plus[k] += 1e-6;
      minus[k] -= 1e-6;
      EXPECT_NEAR(M[k], (work(plus) - work(minus)) / 2e-6, 1e-10) << "psi0 " << psi0 << ", joint " << k + 1;
    }
  }
}

TEST_F(Loaded, JacobianMatchesCentralDifferences)
{
  SolverSettings settings;
  settings.abs_smoothing = 1e-4;  // smooth |theta| so that finite differences are meaningful

  Loads none;
  Loads dead = gravity(0.4);
  dead.tip_force_axial = 0.2;
  dead.tip_force_lateral = 0.3;
  dead.tip_moment = 0.005;
  Loads follower = gravity(kPi);
  follower.tip_force_axial = -0.25;
  follower.tip_force_lateral = 0.15;
  follower.follower = true;
  const std::vector<double> mixed_kappa = {1.0, -1.0, -1.0, 1.0, 1.0, -1.0};

  struct Case
  {
    const Loads* loads;
    std::vector<double> kappa;
  };
  const Case cases[] = {{&none, {}}, {&dead, {}}, {&follower, {}}, {&dead, mixed_kappa}};
  const Tensions F = F6(4, 2, 7, 1, 9, 3);

  for (const Case& c : cases)
  {
    for (unsigned seed : {1u, 2u, 3u})
    {
      const std::vector<double> theta = random_angles(24, 0.28, seed);
      const Eigen::MatrixXd J = jacobian(joints, topology, F, theta, *c.loads, params, settings, c.kappa);
      double worst = 0.0, scale = 0.0;
      for (std::size_t k = 0; k < 24; ++k)
      {
        std::vector<double> plus = theta, minus = theta;
        plus[k] += 1e-7;
        minus[k] -= 1e-7;
        const std::vector<double> gp = residual_vector(joints, topology, F, plus, *c.loads, params, settings, c.kappa);
        const std::vector<double> gm = residual_vector(joints, topology, F, minus, *c.loads, params, settings, c.kappa);
        for (std::size_t i = 0; i < 24; ++i)
        {
          const double fd = (gp[i] - gm[i]) / 2e-7;
          worst = std::max(worst, std::abs(J(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)) - fd));
          scale = std::max(scale, std::abs(fd));
        }
      }
      EXPECT_LT(worst / scale, 1e-6);
    }
  }
}

TEST_F(Loaded, FrictionlessJacobianIsSymmetric)
{
  RobotParams p = params;
  p.mu = 0.0;
  const RollchainModel m(p);
  Loads loads = gravity(0.0);
  loads.tip_force_axial = 0.1;
  loads.tip_force_lateral = 0.2;
  const std::vector<double> theta = random_angles(24, 0.28, 5);
  const Eigen::MatrixXd J =
      jacobian(m.joints(), m.topology(), F6(1, 2, 3, 4, 5, 6), theta, loads, p, m.settings());
  EXPECT_LT((J - J.transpose()).cwiseAbs().maxCoeff(), 1e-12 * J.cwiseAbs().maxCoeff());
}

TEST_F(Loaded, ReferenceCasesEandF)
{
  // Theory §11.2: 3L = 10 N, upright (E) and hanging (F).
  struct Case
  {
    double psi0;
    double theta[6];
    double tip_x, tip_y, tip_angle, pull_3L;
  };
  const Case cases[] = {
      {0.0,
       {0.06943852667676119, 0.0479766244377542, 0.06805573164952708, 0.04406807128484703, 0.08020488174915619,
        0.05858541930470353},
       0.24285333052524813, 0.22569106164667901, 1.440880112006153, 0.003281356187012098},
      {kPi,
       {0.03345575184952052, 0.03018701931358634, 0.04482146686553885, 0.04334441624192461, 0.08191223689866485,
        0.07066408455601725},
       0.3041757626355072, 0.15637862335282868, 1.1221448800993352, 0.002572574759521623},
  };
  const std::size_t joint_index[6] = {0, 9, 10, 17, 18, 23};
  for (const Case& c : cases)
  {
    const State s = model.solve(F6(0, 0, 0, 0, 10, 0), gravity(c.psi0));
    ASSERT_TRUE(s.converged);
    EXPECT_EQ(s.method, SolveMethod::Newton);  // automatic picks Newton with loads
    for (int n = 0; n < 6; ++n)
    {
      EXPECT_NEAR(s.theta[joint_index[n]], c.theta[n], 1e-10);
    }
    const Chain chain = model.kinematics(s);
    EXPECT_NEAR(chain.tip()(0), c.tip_x, 1e-10);
    EXPECT_NEAR(chain.tip()(1), c.tip_y, 1e-10);
    EXPECT_NEAR(chain.tip_angle(), c.tip_angle, 1e-9);
    EXPECT_NEAR(model.pull_in(s)[4], c.pull_3L, 1e-12);
  }
}

TEST_F(Loaded, FixedPointAgreesWithNewton)
{
  const RollchainModel fixed_point(params, with_method(SolveMethod::FixedPoint));
  for (double psi0 : {0.0, kPi})
  {
    const State a = model.solve(F6(0, 0, 0, 0, 10, 0), gravity(psi0));
    const State b = fixed_point.solve(F6(0, 0, 0, 0, 10, 0), gravity(psi0));
    ASSERT_TRUE(b.converged);
    EXPECT_EQ(b.method, SolveMethod::FixedPoint);
    EXPECT_GT(b.iterations, 1);
    for (std::size_t k = 0; k < 24; ++k)
    {
      EXPECT_NEAR(a.theta[k], b.theta[k], 1e-11);
    }
  }
}

TEST_F(Loaded, TipLoadsDeadAndFollower)
{
  Loads dead;
  dead.tip_force_lateral = -0.05;  // pushes the tip to the right
  const State a = model.solve(F6(0, 0, 0, 0, 10, 0), dead);
  ASSERT_TRUE(a.converged);
  const State free = model.solve(F6(0, 0, 0, 0, 10, 0));
  EXPECT_LT(a.theta[0], free.theta[0]);  // the load straightens the bend

  Loads follower = dead;
  follower.follower = true;
  const State b = model.solve(F6(0, 0, 0, 0, 10, 0), follower);
  ASSERT_TRUE(b.converged);
  EXPECT_LT(b.residual, 1e-12);
}

TEST_F(Loaded, UprightStraightRobotIsStable)
{
  // Theory §7.2: frictionless Jacobian at zero tension, upright; smallest
  // eigenvalue 0.146 N m/rad > 0.
  RobotParams p = params;
  p.mu = 0.0;
  const RollchainModel m(p);
  const Eigen::MatrixXd J = jacobian(m.joints(), m.topology(), F6(0, 0, 0, 0, 0, 0), std::vector<double>(24, 0.0),
                                     gravity(0.0), p, m.settings());
  const Eigen::MatrixXd sym = 0.5 * (J + J.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(sym);
  EXPECT_NEAR(eig.eigenvalues().minCoeff(), 0.1457, 5e-4);
}

TEST_F(Loaded, ContactForcesWithoutLoads)
{
  // (RC17), shared turning: F^R = sum T cos(theta/2), H = sum T sin(theta/2).
  const State s = model.solve(F6(0, 0, 0, 0, 10, 0));
  const std::vector<ContactForce> shared = model.contact_forces(s);
  const std::vector<ContactForce> upper = model.contact_forces(s, Loads{}, RodTurning::ByUpperLink);
  for (std::size_t k : {0u, 12u, 23u})
  {
    const double T = s.tension[k][4];
    EXPECT_NEAR(shared[k].normal, T * std::cos(0.5 * s.theta[k]), 1e-12);
    EXPECT_NEAR(shared[k].tangential, T * std::sin(0.5 * s.theta[k]), 1e-12);
    EXPECT_NEAR(upper[k].normal, T, 1e-12);
    EXPECT_NEAR(upper[k].tangential, 0.0, 1e-12);
  }
}

TEST_F(Loaded, Diagnostics)
{
  const State b = model.solve(F6(0, 0, 0, 12, 10, 0));  // case B
  const Diagnostics d = model.diagnostics(b);
  EXPECT_NEAR(d.max_strain, 0.00774, 5e-5);  // theory §9.5: 0.77 %
  EXPECT_GE(d.max_strain_joint, 18u);       // in section 3
  EXPECT_TRUE(d.joints_at_stop.empty());
  EXPECT_GT(d.min_normal_force, 0.0);
  EXPECT_LT(d.max_slip_ratio, std::tan(0.5 * 0.13));
}
