#include <array>
#include <chrono>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tracking_demo");
  const auto topic = node->declare_parameter<std::string>("topic", "/arm/target_pose");
  auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);

  const std::vector<std::array<double, 3>> waypoints = {
    {0.35, 0.00, 0.45},
    {0.35, 0.05, 0.45},
    {0.35, 0.00, 0.50},
    {0.35, -0.05, 0.45},
    {0.35, 0.00, 0.40},
  };

  RCLCPP_INFO(
    node->get_logger(),
    "Publishing low-rate end-effector tracking targets on %s.",
    topic.c_str());

  rclcpp::WallRate rate(0.05);
  std::size_t index = 0;
  while (rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = node->now();
    target.header.frame_id = "world";
    target.pose.position.x = waypoints[index][0];
    target.pose.position.y = waypoints[index][1];
    target.pose.position.z = waypoints[index][2];
    target.pose.orientation.w = 1.0;

    publisher->publish(target);
    RCLCPP_INFO(
      node->get_logger(),
      "Target %zu: %.3f %.3f %.3f",
      index,
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z);

    index = (index + 1) % waypoints.size();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
