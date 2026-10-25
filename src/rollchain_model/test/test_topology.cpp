// Layer 2: sections, joints and the rods crossing each joint (theory §2, RC1).

#include <gtest/gtest.h>

#include "fixtures.hpp"

using namespace rollchain_test;

TEST(Topology, CountsOfTheReferenceRobot)
{
  const Topology t(reference_robot());
  EXPECT_EQ(t.num_joints(), 24u);
  EXPECT_EQ(t.num_sections(), 3u);
  EXPECT_EQ(t.num_rods(), 6u);

  EXPECT_EQ(t.last_link(0), 0u);
  EXPECT_EQ(t.last_link(1), 10u);
  EXPECT_EQ(t.last_link(2), 18u);
  EXPECT_EQ(t.last_link(3), 24u);
}

TEST(Topology, SectionOfEachJoint)
{
  const Topology t(reference_robot());
  EXPECT_EQ(t.section_of(0), 1u);
  EXPECT_EQ(t.section_of(9), 1u);
  EXPECT_EQ(t.section_of(10), 2u);
  EXPECT_EQ(t.section_of(17), 2u);
  EXPECT_EQ(t.section_of(18), 3u);
  EXPECT_EQ(t.section_of(23), 3u);
}

TEST(Topology, RodsCrossingEachJointAreATailOfTheList)
{
  const Topology t(reference_robot());
  // Theory §2.3: 6, 4 and 2 rods cross the joints of sections 1, 2, 3.
  EXPECT_EQ(t.first_rod_at(0), 0u);
  EXPECT_EQ(t.first_rod_at(9), 0u);
  EXPECT_EQ(t.first_rod_at(10), 2u);
  EXPECT_EQ(t.first_rod_at(17), 2u);
  EXPECT_EQ(t.first_rod_at(18), 4u);
  EXPECT_EQ(t.first_rod_at(23), 4u);
  EXPECT_EQ(t.first_rod(4), 6u);

  for (std::size_t k = 0; k < t.num_joints(); ++k)
  {
    for (std::size_t j = 0; j < t.num_rods(); ++j)
    {
      EXPECT_EQ(t.crosses(j, k), j >= t.first_rod_at(k)) << "rod " << j << ", joint " << k;
    }
  }
}

TEST(Topology, AnchorIsTheLastLinkOfTheRodsSection)
{
  const Topology t(reference_robot());
  const std::size_t expected[6] = {10, 10, 18, 18, 24, 24};
  for (std::size_t j = 0; j < 6; ++j)
  {
    EXPECT_EQ(t.anchor(j), expected[j]);
  }
}

TEST(Topology, SectionWithoutOwnRods)
{
  // Section 1 has no rods of its own; it is crossed by the section-2 rods.
  RobotParams p = reference_robot();
  p.sections.resize(2);
  p.rods.erase(p.rods.begin(), p.rods.begin() + 2);  // drop 1L, 1R
  p.rods.resize(2);                                  // keep 2L, 2R
  ASSERT_NO_THROW(validate(p));
  const Topology t(p);
  EXPECT_EQ(t.first_rod(1), 0u);
  EXPECT_EQ(t.first_rod(2), 0u);
  EXPECT_EQ(t.first_rod(3), 2u);
  EXPECT_EQ(t.first_rod_at(0), 0u);
  EXPECT_EQ(t.anchor(0), 18u);
}
