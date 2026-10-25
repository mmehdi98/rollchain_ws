// Layer 5: joint balance and the joint-by-joint solver (theory §6.5, §7.1,
// §11.1, §11.2).

#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "fixtures.hpp"

using namespace rollchain_test;

namespace
{

constexpr double kAngleTol = 1e-11;  // [rad]

// Reference values (theory §11.2), computed with the Python implementation
// of the theory: theta at joints 1, 10, 11, 18, 19, 24 [rad], tip (X, Y) [m],
// tip angle [rad], pull-in of rod 3L [m].
struct Reference
{
  Tensions F;
  double theta[6];
  double tip_x, tip_y, tip_angle, pull_3L;
};

const std::size_t kJoints[6] = {0, 9, 10, 17, 18, 23};

void expect_matches(const RollchainModel& model, const State& state, const Reference& ref)
{
  ASSERT_TRUE(state.converged);
  for (int n = 0; n < 6; ++n)
  {
    EXPECT_NEAR(state.theta[kJoints[n]], ref.theta[n], kAngleTol) << "joint " << kJoints[n] + 1;
  }
  const Chain chain = model.kinematics(state);
  EXPECT_NEAR(chain.tip()(0), ref.tip_x, 1e-11);
  EXPECT_NEAR(chain.tip()(1), ref.tip_y, 1e-11);
  EXPECT_NEAR(chain.tip_angle(), ref.tip_angle, 1e-10);
  EXPECT_NEAR(model.pull_in(state)[4], ref.pull_3L, 1e-12);
}

class Statics : public ::testing::Test
{
protected:
  RollchainModel model{reference_robot()};
};

}  // namespace

TEST_F(Statics, ZeroTensionGivesTheStraightRobot)
{
  const State s = model.solve(F6(0, 0, 0, 0, 0, 0));
  ASSERT_EQ(s.theta.size(), 24u);
  for (double th : s.theta)
  {
    EXPECT_DOUBLE_EQ(th, 0.0);
  }
  EXPECT_TRUE(s.converged);
  EXPECT_EQ(s.method, SolveMethod::Sequential);
}

TEST_F(Statics, NegativeTensionIsSlack)
{
  const State a = model.solve(F6(0, -5, 0, 0, 10, 0));
  const State b = model.solve(F6(0, 0, 0, 0, 10, 0));
  for (std::size_t k = 0; k < 24; ++k)
  {
    EXPECT_DOUBLE_EQ(a.theta[k], b.theta[k]);
  }
}

TEST_F(Statics, WrongNumberOfTensionsThrows)
{
  EXPECT_THROW(model.solve({1.0, 2.0}), std::invalid_argument);
  EXPECT_THROW(model.solve(F6(0, 0, 0, 0, 1, 0), Loads{}, {1.0}), std::invalid_argument);
}

TEST_F(Statics, OnlySectionOneRodsLeaveTheRestStraight)
{
  const State s = model.solve(F6(12, 3, 0, 0, 0, 0));
  EXPECT_GT(s.theta[0], 0.0);
  for (std::size_t k = 10; k < 24; ++k)
  {
    EXPECT_EQ(s.theta[k], 0.0) << "joint " << k + 1;
  }
}

TEST_F(Statics, DecouplingKeepsProximalSectionsStraight)
{
  // Theory §6.5: F_2R = (a3 / a2) F_3L = 1.2 F_3L bends only section 3.
  const State s = model.solve(F6(0, 0, 0, 12.0, 10.0, 0));
  for (std::size_t k = 0; k < 18; ++k)
  {
    EXPECT_NEAR(s.theta[k], 0.0, 1e-14) << "joint " << k + 1;
  }
  EXPECT_NEAR(s.theta[18], 0.12708164053113868, kAngleTol);
}

TEST_F(Statics, ScaleInvariance)
{
  // (F, E) -> lambda (F, E) leaves the angles unchanged (no loads).
  RobotParams p = reference_robot();
  for (RodParams& rod : p.rods)
  {
    rod.E *= 3.7;
  }
  const RollchainModel stiff(p);
  const Tensions F = F6(4, 0, 0, 7, 9, 1);
  Tensions F_scaled = F;
  for (double& f : F_scaled)
  {
    f *= 3.7;
  }
  const State a = model.solve(F);
  const State b = stiff.solve(F_scaled);
  for (std::size_t k = 0; k < 24; ++k)
  {
    EXPECT_NEAR(a.theta[k], b.theta[k], 1e-13);
  }
}

TEST_F(Statics, MirrorSymmetry)
{
  const State a = model.solve(F6(4, 0, 0, 7, 9, 1));
  const State b = model.solve(F6(0, 4, 7, 0, 1, 9));
  for (std::size_t k = 0; k < 24; ++k)
  {
    EXPECT_NEAR(a.theta[k], -b.theta[k], 1e-14);
  }
}

TEST_F(Statics, PassiveCablesEqualASingleSection)
{
  // Rods of sections 1 and 2 without bending stiffness: pulling 3L must give
  // the same angles as one 24-link section with the 3L/3R pair only.
  RobotParams p = reference_robot();
  for (std::size_t j = 0; j < 4; ++j)
  {
    p.rods[j].E = 0.0;
  }
  const RollchainModel cables(p);

  RobotParams q = reference_robot();
  q.sections.resize(1);
  q.sections[0].n = 24;
  q.rods = {p.rods[4], p.rods[5]};
  q.rods[0].section = 1;
  q.rods[1].section = 1;
  const RollchainModel single(q);

  const State a = cables.solve(F6(0, 0, 0, 0, 8.0, 0));
  const State b = single.solve({8.0, 0.0});
  for (std::size_t k = 0; k < 24; ++k)
  {
    EXPECT_NEAR(a.theta[k], b.theta[k], 1e-13);
  }
}

TEST_F(Statics, SmallAngleForm)
{
  // (RC15): theta ~ eta Q / (k0 + eta P R) at small load.
  const Tensions F = F6(0, 0, 0.05, 0, 0, 0.03);
  const State s = model.solve(F);
  for (std::size_t k = 0; k < 24; ++k)
  {
    const Joint& j = model.joints()[k];
    double P = 0.0, Q = 0.0;
    for (const RodAtJoint& rod : j.rods)
    {
      P += F[rod.rod];
      Q += F[rod.rod] * rod.u;
    }
    // eta from the stored tensions of a tensioned rod
    const std::size_t probe = k < 18 ? 2 : 5;
    const double eta = s.tension[k][probe] / F[probe];
    const double linear = eta * Q / (j.k0 + eta * P * j.R);
    EXPECT_NEAR(s.theta[k], linear, 1e-6 * std::abs(linear) + 1e-12);
  }
}

TEST_F(Statics, TensionTable)
{
  // Case A: 3L = 10 N. Theory §9.4: 76 % reaches section 2, 59 % section 3.
  const State a = model.solve(F6(0, 0, 0, 0, 10.0, 0));
  ASSERT_EQ(a.tension.size(), 25u);
  EXPECT_DOUBLE_EQ(a.tension[0][4], 10.0);
  EXPECT_NEAR(a.tension[10][4], 7.588848621008666, 1e-11);
  EXPECT_NEAR(a.tension[18][4], 5.868706995506748, 1e-11);
  EXPECT_NEAR(a.tension[24][4], 4.39142794423218, 1e-11);

  // Rods are zero above their anchor link.
  const State c = model.solve(F6(5, 0, 5, 0, 5, 0));
  EXPECT_GT(c.tension[10][0], 0.0);
  EXPECT_EQ(c.tension[11][0], 0.0);
  EXPECT_GT(c.tension[18][2], 0.0);
  EXPECT_EQ(c.tension[19][2], 0.0);
}

TEST_F(Statics, ReferenceCasesAtoD)
{
  const Reference cases[] = {
      // A: 3L = 10
      {F6(0, 0, 0, 0, 10, 0),
       {0.047038629959882484, 0.03722704321422175, 0.05409473781140823, 0.043860974991343546,
        0.08161161330729906, 0.06570712038900606},
       0.2828003008309333, 0.18551206429310843, 1.2468646163057269, 0.0028545706167424625},
      // B: 3L = 10, 2R = 12
      {F6(0, 0, 0, 12, 10, 0),
       {0.0, 0.0, 0.0, 0.0, 0.12708164053113868, 0.09377879604866816},
       0.3516305365386763, 0.034660927335212324, 0.655277318819026, 0.001428135287766038},
      // C: 1L = 2L = 3L = 5
      {F6(5, 0, 5, 0, 5, 0),
       {0.05659248975684869, 0.04327150927589882, 0.04735738374049399, 0.03931772174835944,
        0.042908960029344446, 0.03784447135296879},
       0.28421633866396456, 0.19284088884213804, 1.0811058245101564, 0.0024960521535215816},
      // D: 3L = 15, 3R = 5
      {F6(0, 0, 0, 0, 15, 5),
       {0.043617458266210815, 0.035522821512280575, 0.05029651101191179, 0.04192910329663301,
        0.07353836356686647, 0.06170237136344202},
       0.29157815358781164, 0.1765813229276695, 1.165015028099738, 0.0026764503352247563},
  };
  for (const Reference& ref : cases)
  {
    expect_matches(model, model.solve(ref.F), ref);
  }
}

TEST(OneSection, MatchesLopsyWithSymmetricLinks)
{
  // Theory §11.1 / architecture §5: lopsy_kinematics with A1 = A2 = 2.2 mm and
  // R1 = R2 = 4 mm reproduces the smoothed values to 7e-13 rad.
  struct Case
  {
    double FL, FR, th1, th15_exact, th15_smoothed;
  };
  const Case cases[] = {
      {10, 0, 0.11399403168905066, 0.059337991113575836, 0.05937569182635603},
      {0, 7, -0.08499328496763181, -0.04976931005584923, -0.049802902803590954},
      {12, 3, 0.09291291683503448, 0.05506171286706702, 0.05509581783304524},
      {25, 0, 0.21878326887100052, 0.08435735834191502, 0.08440338773241131},
  };
  const RollchainModel exact(one_section_robot());
  SolverSettings smooth;
  smooth.abs_smoothing = 1e-4;
  const RollchainModel smoothed(one_section_robot(), smooth);
  for (const Case& c : cases)
  {
    const State a = exact.solve({c.FL, c.FR});
    EXPECT_NEAR(a.theta.front(), c.th1, kAngleTol);
    EXPECT_NEAR(a.theta.back(), c.th15_exact, kAngleTol);
    const State b = smoothed.solve({c.FL, c.FR});
    EXPECT_NEAR(b.theta.front(), c.th1, kAngleTol);
    EXPECT_NEAR(b.theta.back(), c.th15_smoothed, kAngleTol);
  }
}

TEST_F(Statics, SequentialMethodRejectsLoads)
{
  const RollchainModel seq(reference_robot(), with_method(SolveMethod::Sequential));
  Loads loads;
  loads.gravity_in_plane = true;
  EXPECT_THROW(seq.solve(F6(0, 0, 0, 0, 10, 0), loads), std::invalid_argument);
  EXPECT_NO_THROW(seq.solve(F6(0, 0, 0, 0, 10, 0)));
}

TEST_F(Statics, ResidualIsTinyAndNoStopIsEngaged)
{
  const State s = model.solve(F6(5, 0, 5, 0, 5, 0));
  EXPECT_LT(s.residual, 1e-14);
  EXPECT_TRUE(model.diagnostics(s).joints_at_stop.empty());
}
