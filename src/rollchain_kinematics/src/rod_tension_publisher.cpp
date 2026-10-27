// rod_tension_publisher: publishes one tension per rod from parameters, for
// use without the slider GUI (for example with rqt_reconfigure).
//
//   parameters  rods.name       from the robot config: one tension parameter per rod
//               tension.<name>  base tension of that rod [N] (default 0)
//               max_tension     slider range in rqt_reconfigure [N] (default 60)
//               publish_rate    [Hz] (default 20)
//   publishes   rod_tensions    (rollchain_interfaces/RodTensions, with names)

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rollchain_interfaces/msg/rod_tensions.hpp"

using RodTensions = rollchain_interfaces::msg::RodTensions;

class RodTensionPublisher : public rclcpp::Node
{
public:
  RodTensionPublisher() : Node("rod_tension_publisher")
  {
    rcl_interfaces::msg::ParameterDescriptor names_desc;
    names_desc.description = "rod names (from the robot config)";
    names_desc.read_only = true;
    rod_names_ = declare_parameter<std::vector<std::string>>("rods.name", names_desc);

    const double max_tension = declare_parameter("max_tension", 60.0);
    const double rate = declare_parameter("publish_rate", 20.0);

    for (const std::string& name : rod_names_)
    {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = "base tension of rod " + name + " [N]";
      rcl_interfaces::msg::FloatingPointRange range;
      range.from_value = 0.0;
      range.to_value = max_tension;
      range.step = 0.0;  // continuous
      desc.floating_point_range.push_back(range);
      declare_parameter("tension." + name, 0.0, desc);
    }

    publisher_ = create_publisher<RodTensions>("rod_tensions", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / std::max(rate, 1e-3)), [this]() { publish(); });
    RCLCPP_INFO(get_logger(), "publishing tensions of %zu rods", rod_names_.size());
  }

private:
  void publish()
  {
    RodTensions msg;
    msg.header.stamp = now();
    msg.name = rod_names_;
    msg.tension.reserve(rod_names_.size());
    for (const std::string& name : rod_names_)
    {
      msg.tension.push_back(get_parameter("tension." + name).as_double());
    }
    publisher_->publish(msg);
  }

  std::vector<std::string> rod_names_;
  rclcpp::Publisher<RodTensions>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try
  {
    rclcpp::spin(std::make_shared<RodTensionPublisher>());
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("rod_tension_publisher"), "cannot start: %s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
