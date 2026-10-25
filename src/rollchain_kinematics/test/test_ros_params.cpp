// The parameter adapters: robot config parameters -> rollchain_model structs.

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rollchain_kinematics/ros_params.hpp"
#include "rollchain_model/model.hpp"

using rollchain_kinematics::apply_parameter;
using rollchain_kinematics::declare_robot_parameters;
using rollchain_kinematics::NodeConfig;

namespace
{

class RosParams : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  /// The reference robot of config/reference_robot.yaml, as overrides.
  static std::vector<rclcpp::Parameter> reference()
  {
    return {
        rclcpp::Parameter("base.length", 0.015),
        rclcpp::Parameter("sections.n", std::vector<int64_t>{10, 8, 6}),
        rclcpp::Parameter("sections.L", std::vector<double>{0.015, 0.015, 0.015}),
        rclcpp::Parameter("sections.R", std::vector<double>{0.004, 0.004, 0.004}),
        rclcpp::Parameter("sections.D", std::vector<double>{0.0062, 0.0062, 0.0062}),
        rclcpp::Parameter("sections.weight", std::vector<double>{5.685e-3, 5.685e-3, 5.685e-3}),
        rclcpp::Parameter("sections.y_cg", std::vector<double>{7.35e-3, 7.35e-3, 7.35e-3}),
        rclcpp::Parameter("rods.name", std::vector<std::string>{"1L", "1R", "2L", "2R", "3L", "3R"}),
        rclcpp::Parameter("rods.section", std::vector<int64_t>{1, 1, 2, 2, 3, 3}),
        rclcpp::Parameter("rods.x", std::vector<double>{1.6e-3, -1.6e-3, 2.0e-3, -2.0e-3, 2.4e-3, -2.4e-3}),
        rclcpp::Parameter("rods.r", std::vector<double>(6, 0.3e-3)),
        rclcpp::Parameter("rods.E", std::vector<double>(6, 61.0e9)),
        rclcpp::Parameter("holes.lambda0", 2.2e-3),
        rclcpp::Parameter("holes.r_h", 0.4e-3),
        rclcpp::Parameter("friction.mu", 0.66),
    };
  }

  static std::shared_ptr<rclcpp::Node> make_node(const std::vector<rclcpp::Parameter>& overrides)
  {
    static int count = 0;
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    return std::make_shared<rclcpp::Node>("test_ros_params_" + std::to_string(count++), options);
  }

  static std::vector<rclcpp::Parameter> without(const std::string& name)
  {
    std::vector<rclcpp::Parameter> out;
    for (const rclcpp::Parameter& p : reference())
    {
      if (p.get_name() != name)
      {
        out.push_back(p);
      }
    }
    return out;
  }
};

}  // namespace

TEST_F(RosParams, ReadsTheReferenceConfig)
{
  auto node = make_node(reference());
  const NodeConfig c = declare_robot_parameters(*node);
  ASSERT_EQ(c.robot.sections.size(), 3u);
  EXPECT_EQ(c.robot.sections[1].n, 8u);
  EXPECT_DOUBLE_EQ(c.robot.sections[2].R, 0.004);
  EXPECT_DOUBLE_EQ(c.robot.sections[0].weight, 5.685e-3);
  ASSERT_EQ(c.robot.rods.size(), 6u);
  EXPECT_EQ(c.robot.rods[4].name, "3L");
  EXPECT_EQ(c.robot.rods[4].section, 3u);
  EXPECT_DOUBLE_EQ(c.robot.rods[5].x, -2.4e-3);
  EXPECT_DOUBLE_EQ(c.robot.rods[0].E, 61.0e9);
  EXPECT_DOUBLE_EQ(c.robot.mu, 0.66);
  EXPECT_FALSE(c.loads.gravity_in_plane);
  EXPECT_EQ(c.solver.method, rollchain_model::SolveMethod::Automatic);

  const rollchain_model::RollchainModel model(c.robot, c.solver);
  EXPECT_EQ(model.num_joints(), 24u);
}

TEST_F(RosParams, OptionalPhysicsHaveDefaults)
{
  std::vector<rclcpp::Parameter> overrides;
  for (const rclcpp::Parameter& p : reference())
  {
    const std::string& n = p.get_name();
    if (n != "sections.weight" && n != "sections.y_cg" && n != "rods.r" && n != "rods.E")
    {
      overrides.push_back(p);
    }
  }
  auto node = make_node(overrides);
  const NodeConfig c = declare_robot_parameters(*node);
  EXPECT_DOUBLE_EQ(c.robot.sections[0].weight, 0.0);
  EXPECT_DOUBLE_EQ(c.robot.sections[0].y_cg, 0.0075);  // L / 2
  EXPECT_DOUBLE_EQ(c.robot.rods[3].r, 0.3e-3);
  EXPECT_DOUBLE_EQ(c.robot.rods[3].E, 61.0e9);
}

TEST_F(RosParams, MissingStructureParameterThrows)
{
  auto node = make_node(without("rods.x"));
  EXPECT_ANY_THROW(declare_robot_parameters(*node));
}

TEST_F(RosParams, MismatchedArraysThrow)
{
  auto overrides = without("rods.x");
  overrides.emplace_back("rods.x", std::vector<double>{1.6e-3, -1.6e-3, 2.0e-3, -2.0e-3, 2.4e-3});
  auto node = make_node(overrides);
  EXPECT_THROW(declare_robot_parameters(*node), std::invalid_argument);
}

TEST_F(RosParams, ApplyParameter)
{
  auto node = make_node(reference());
  NodeConfig c = declare_robot_parameters(*node);
  std::string reason;

  EXPECT_TRUE(apply_parameter(c, rclcpp::Parameter("friction.mu", 0.5), reason));
  EXPECT_DOUBLE_EQ(c.robot.mu, 0.5);

  EXPECT_TRUE(apply_parameter(c, rclcpp::Parameter("gravity.enabled", true), reason));
  EXPECT_TRUE(c.loads.gravity_in_plane);

  EXPECT_TRUE(apply_parameter(c, rclcpp::Parameter("rods.E", std::vector<double>(6, 50.0e9)), reason));
  EXPECT_DOUBLE_EQ(c.robot.rods[2].E, 50.0e9);

  EXPECT_FALSE(apply_parameter(c, rclcpp::Parameter("rods.E", std::vector<double>(4, 50.0e9)), reason));
  EXPECT_NE(reason.find("rods.E"), std::string::npos);

  EXPECT_FALSE(apply_parameter(c, rclcpp::Parameter("sections.n", std::vector<int64_t>{10, 8}), reason));
  EXPECT_NE(reason.find("structure"), std::string::npos);

  EXPECT_FALSE(apply_parameter(c, rclcpp::Parameter("solver.method", std::string("bogus")), reason));
  EXPECT_TRUE(apply_parameter(c, rclcpp::Parameter("solver.method", std::string("fixed_point")), reason));
  EXPECT_EQ(c.solver.method, rollchain_model::SolveMethod::FixedPoint);

  EXPECT_TRUE(apply_parameter(c, rclcpp::Parameter("use_sim_time", false), reason));  // not ours
}
