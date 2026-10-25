// Layer 1: parameters and their validation (theory §4.5).

#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "fixtures.hpp"

using namespace rollchain_test;

namespace
{

/// Expects validate() to throw, with `keyword` somewhere in the message.
void expect_invalid(const RobotParams& p, const std::string& keyword)
{
  try
  {
    validate(p);
    ADD_FAILURE() << "validate() accepted parameters that should fail with: " << keyword;
  }
  catch (const std::invalid_argument& e)
  {
    EXPECT_NE(std::string(e.what()).find(keyword), std::string::npos)
        << "message does not mention '" << keyword << "':\n" << e.what();
  }
}

}  // namespace

TEST(Params, ReferenceRobotIsValid)
{
  EXPECT_NO_THROW(validate(reference_robot()));
  EXPECT_NO_THROW(validate(one_section_robot()));
  EXPECT_NO_THROW(validate(SolverSettings{}));
}

TEST(Params, RejectsMissingSectionsOrRods)
{
  RobotParams p = reference_robot();
  p.rods.clear();
  expect_invalid(p, "at least one rod");

  p = reference_robot();
  p.sections.clear();
  expect_invalid(p, "at least one section");
}

TEST(Params, RejectsRodsOutOfSectionOrder)
{
  RobotParams p = reference_robot();
  std::swap(p.rods[1], p.rods[4]);  // a section-3 rod before a section-2 rod
  expect_invalid(p, "section by section");
}

TEST(Params, RejectsDuplicateNames)
{
  RobotParams p = reference_robot();
  p.rods[3].name = "1L";
  expect_invalid(p, "duplicate");
}

TEST(Params, RejectsUnknownSection)
{
  RobotParams p = reference_robot();
  p.rods[5].section = 4;
  expect_invalid(p, "does not exist");
}

TEST(Params, RejectsRodOnTheContactPath)
{
  RobotParams p = reference_robot();
  p.rods[0].x = 0.3e-3;  // |x| <= r_h: the joint range would be empty
  expect_invalid(p, "hole radius");
}

TEST(Params, RejectsHoleOutsideTheLink)
{
  RobotParams p = reference_robot();
  p.rods[5].x = -2.9e-3;  // 2.9 + 0.4 + 0.2 > 3.1
  expect_invalid(p, "does not fit");

  p = reference_robot();
  p.rods[5].x = -5.0e-3;  // beyond the arc radius as well
  expect_invalid(p, "on the arc");
}

TEST(Params, RejectsBadSections)
{
  RobotParams p = reference_robot();
  p.sections[1].n = 0;
  expect_invalid(p, "n must be >= 1");

  p = reference_robot();
  p.sections[2].R = 3.0e-3;  // < D/2
  expect_invalid(p, "spans the link");

  p = reference_robot();
  p.sections[0].y_cg = 20.0e-3;  // > L
  expect_invalid(p, "y_cg");
}

TEST(Params, RejectsLongSetback)
{
  RobotParams p = reference_robot();
  p.lambda0 = 8.0e-3;  // more than half of a ~14.3 mm hole
  expect_invalid(p, "half the hole length");
}

TEST(Params, RejectsJointsWithoutStiffness)
{
  RobotParams p = reference_robot();
  p.rods[4].E = 0.0;
  p.rods[5].E = 0.0;  // nothing stiff crosses section 3
  expect_invalid(p, "no rod with E > 0");

  // Cables (E = 0) are fine where a stiff rod also crosses.
  p = reference_robot();
  p.rods[0].E = 0.0;
  EXPECT_NO_THROW(validate(p));
}

TEST(Params, ReportsEveryProblemAtOnce)
{
  RobotParams p = reference_robot();
  p.rods[3].name = "1L";
  p.sections[1].n = 0;
  try
  {
    validate(p);
    FAIL() << "expected std::invalid_argument";
  }
  catch (const std::invalid_argument& e)
  {
    const std::string what = e.what();
    EXPECT_NE(what.find("duplicate"), std::string::npos);
    EXPECT_NE(what.find("n must be >= 1"), std::string::npos);
  }
}

TEST(Params, SolverSettingsValidation)
{
  SolverSettings s;
  s.load_steps = 0;
  EXPECT_THROW(validate(s), std::invalid_argument);
  s = SolverSettings{};
  s.angle_tolerance = 0.0;
  EXPECT_THROW(validate(s), std::invalid_argument);
}

TEST(Params, SolveMethodNames)
{
  for (SolveMethod m : {SolveMethod::Automatic, SolveMethod::Sequential, SolveMethod::FixedPoint, SolveMethod::Newton})
  {
    EXPECT_EQ(solve_method_from_string(to_string(m)), m);
  }
  EXPECT_THROW(solve_method_from_string("gauss_seidel"), std::invalid_argument);
}

TEST(Params, LoadsAny)
{
  Loads loads;
  EXPECT_FALSE(loads.any());
  loads.gravity_in_plane = true;
  EXPECT_TRUE(loads.any());
  loads.scale = 0.0;
  EXPECT_FALSE(loads.any());
  loads = Loads{};
  loads.tip_moment = 1e-3;
  EXPECT_TRUE(loads.any());
}
