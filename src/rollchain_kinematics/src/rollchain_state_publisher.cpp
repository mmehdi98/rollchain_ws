// rollchain_state_publisher: rod tensions in, joint states and rod state out.
//
//   subscribes  rod_tensions  (rollchain_interfaces/RodTensions)
//               tip_wrench    (geometry_msgs/WrenchStamped; frame "tip" = follower load, otherwise base frame)
//   publishes   joint_states  (sensor_msgs/JointState: j1_roll..jN_roll, for robot_state_publisher)
//               rod_state     (rollchain_interfaces/RodState)
//
// Parameters: the robot config (see ros_params.hpp) plus publish_rate.
// Physics, gravity and solver parameters can change at runtime: the node
// builds a new model and rejects the change if the new model is invalid.

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rollchain_interfaces/msg/rod_state.hpp"
#include "rollchain_interfaces/msg/rod_tensions.hpp"
#include "rollchain_kinematics/ros_params.hpp"
#include "rollchain_model/model.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace rollchain_kinematics
{

namespace rm = rollchain_model;
using JointState = sensor_msgs::msg::JointState;
using RodState = rollchain_interfaces::msg::RodState;
using RodTensions = rollchain_interfaces::msg::RodTensions;
using WrenchStamped = geometry_msgs::msg::WrenchStamped;

class RollchainStatePublisher : public rclcpp::Node
{
public:
  RollchainStatePublisher() : Node("rollchain_state_publisher")
  {
    // Both throw on a bad config; main() logs the reason and exits.
    config_ = declare_robot_parameters(*this);
    model_ = std::make_shared<const rm::RollchainModel>(config_.robot, config_.solver);
    const double rate = declare_parameter("publish_rate", 10.0);

    joint_names_ = rm::urdf_joint_names(model_->num_joints());
    tensions_.assign(model_->num_rods(), 0.0);
    for (std::size_t j = 0; j < config_.robot.rods.size(); ++j)
    {
      rod_names_.push_back(config_.robot.rods[j].name);
      rod_index_[rod_names_.back()] = j;
    }

    joint_pub_ = create_publisher<JointState>("joint_states", 10);
    rod_pub_ = create_publisher<RodState>("rod_state", 10);
    tension_sub_ = create_subscription<RodTensions>(
        "rod_tensions", 10, [this](const RodTensions& msg) { on_tensions(msg); });
    wrench_sub_ = create_subscription<WrenchStamped>(
        "tip_wrench", 10, [this](const WrenchStamped& msg) { on_wrench(msg); });
    // Registered after every declaration: declaring a parameter also calls it.
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& p) { return on_parameters(p); });

    // Re-publish the last joint states regularly, so robot_state_publisher and
    // RViz have a TF tree even when no tension message arrives.
    if (rate > 0.0)
    {
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { republish(); });
    }

    solve_and_publish();
    RCLCPP_INFO(get_logger(), "rollchain model ready: %zu sections, %zu joints, %zu rods",
                model_->topology().num_sections(), model_->num_joints(), model_->num_rods());
  }

private:
  void on_tensions(const RodTensions& msg)
  {
    if (msg.name.empty())
    {
      if (msg.tension.size() != tensions_.size())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "rod_tensions without names must have %zu entries (got %zu); ignored",
                             tensions_.size(), msg.tension.size());
        return;
      }
      tensions_ = msg.tension;
    }
    else
    {
      if (msg.name.size() != msg.tension.size())
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "rod_tensions has %zu names but %zu tensions; ignored", msg.name.size(),
                             msg.tension.size());
        return;
      }
      for (const std::string& name : msg.name)
      {
        if (rod_index_.find(name) == rod_index_.end())
        {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "unknown rod '%s' in rod_tensions; ignored",
                               name.c_str());
          return;
        }
      }
      for (std::size_t i = 0; i < msg.name.size(); ++i)
      {
        tensions_[rod_index_.at(msg.name[i])] = msg.tension[i];
      }
    }
    solve_and_publish();
  }

  void on_wrench(const WrenchStamped& msg)
  {
    // URDF axes: z axial, x lateral (+ = left); a torque about +y bends left.
    const std::string& frame = msg.header.frame_id;
    if (!frame.empty() && frame != "tip" && frame != "base_link")
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "tip_wrench frame '%s' not supported; treating it as base_link", frame.c_str());
    }
    config_.loads.follower = frame == "tip";
    config_.loads.tip_force_axial = msg.wrench.force.z;
    config_.loads.tip_force_lateral = msg.wrench.force.x;
    config_.loads.tip_moment = msg.wrench.torque.y;
    solve_and_publish();
  }

  void solve_and_publish()
  {
    rm::State state;
    try
    {
      state = model_->solve(tensions_, config_.loads);
    }
    catch (const std::exception& e)  // e.g. solver.method = sequential with gravity on
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cannot solve: %s", e.what());
      return;
    }
    RodState rs;
    rs.header.stamp = now();
    rs.name = rod_names_;
    rs.tension_base = state.tension.front();
    rs.converged = state.converged;
    rs.residual = state.residual;

    if (!state.converged)
    {
      // Report it (the slider GUI shows it), but keep showing the last good shape.
      rod_pub_->publish(rs);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "static solve did not converge (residual %.3g N m); keeping the last state",
                           state.residual);
      return;
    }

    last_joints_.name = joint_names_;
    last_joints_.position = state.theta;
    republish();

    rs.tension_anchor.resize(model_->num_rods());
    for (std::size_t j = 0; j < model_->num_rods(); ++j)
    {
      rs.tension_anchor[j] = state.tension[model_->topology().anchor(j)][j];
    }
    rs.pull_in = model_->pull_in(state);
    const rm::Diagnostics d = model_->diagnostics(state, config_.loads);
    rs.stable = d.stable;
    rs.stability_margin = d.stability_margin;
    rs.max_strain = d.max_strain;
    for (std::size_t k : d.joints_at_stop)
    {
      rs.joints_at_stop.push_back(static_cast<uint32_t>(k + 1));
    }
    rs.min_normal_force = d.min_normal_force;
    rs.max_slip_ratio = d.max_slip_ratio;
    rod_pub_->publish(rs);

    if (!d.joints_at_stop.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%zu joints are at their stop (first: joint %zu)",
                           d.joints_at_stop.size(), d.joints_at_stop.front() + 1);
    }
    if (!d.stable)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "unstable equilibrium (stiffness margin %.3g N m/rad): the real robot would buckle",
                           d.stability_margin);
    }
  }

  void republish()
  {
    if (last_joints_.name.empty())
    {
      return;
    }
    last_joints_.header.stamp = now();
    joint_pub_->publish(last_joints_);
  }

  rcl_interfaces::msg::SetParametersResult on_parameters(const std::vector<rclcpp::Parameter>& parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    NodeConfig candidate = config_;
    for (const rclcpp::Parameter& p : parameters)
    {
      std::string reason;
      if (!apply_parameter(candidate, p, reason))
      {
        result.successful = false;
        result.reason = reason;
        return result;
      }
    }
    if (candidate.solver.method == rm::SolveMethod::Sequential && candidate.loads.any())
    {
      result.successful = false;
      result.reason = "the sequential solver cannot handle gravity or tip loads";
      return result;
    }
    try
    {
      // A single-threaded executor never runs this callback during a solve,
      // so swapping the model here is safe.
      model_ = std::make_shared<const rm::RollchainModel>(candidate.robot, candidate.solver);
      config_ = candidate;
    }
    catch (const std::exception& e)
    {
      result.successful = false;
      result.reason = e.what();
      return result;
    }
    result.successful = true;
    solve_and_publish();
    return result;
  }

  NodeConfig config_;
  std::shared_ptr<const rm::RollchainModel> model_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> rod_names_;
  std::unordered_map<std::string, std::size_t> rod_index_;
  rm::Tensions tensions_;
  JointState last_joints_;

  rclcpp::Publisher<JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<RodState>::SharedPtr rod_pub_;
  rclcpp::Subscription<RodTensions>::SharedPtr tension_sub_;
  rclcpp::Subscription<WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace rollchain_kinematics

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try
  {
    rclcpp::spin(std::make_shared<rollchain_kinematics::RollchainStatePublisher>());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("rollchain_state_publisher"), "cannot start: %s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
