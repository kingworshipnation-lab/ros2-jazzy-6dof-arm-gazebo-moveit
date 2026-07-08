#include <cstdlib>
#include <iostream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
double argOrDefault(char ** argv, int index, double fallback)
{
  return argv[index] == nullptr ? fallback : std::atof(argv[index]);
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("send_pose");
  const auto topic = node->declare_parameter<std::string>("topic", "/arm/target_pose");
  auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "world";
  target.pose.position.x = argc > 1 ? argOrDefault(argv, 1, 0.35) : 0.35;
  target.pose.position.y = argc > 2 ? argOrDefault(argv, 2, 0.0) : 0.0;
  target.pose.position.z = argc > 3 ? argOrDefault(argv, 3, 0.45) : 0.45;
  target.pose.orientation.x = argc > 4 ? argOrDefault(argv, 4, 0.0) : 0.0;
  target.pose.orientation.y = argc > 5 ? argOrDefault(argv, 5, 0.0) : 0.0;
  target.pose.orientation.z = argc > 6 ? argOrDefault(argv, 6, 0.0) : 0.0;
  target.pose.orientation.w = argc > 7 ? argOrDefault(argv, 7, 1.0) : 1.0;

  if (argc != 1 && argc != 8) {
    std::cerr << "Usage: ros2 run arm_controller send_pose [x y z qx qy qz qw]\n";
    rclcpp::shutdown();
    return 2;
  }

  rclcpp::WallRate wait_rate(20.0);
  for (int i = 0; rclcpp::ok() && publisher->get_subscription_count() == 0 && i < 40; ++i) {
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  target.header.stamp = node->now();
  publisher->publish(target);
  rclcpp::spin_some(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Published target pose %.3f %.3f %.3f on %s.",
    target.pose.position.x,
    target.pose.position.y,
    target.pose.position.z,
    topic.c_str());

  rclcpp::shutdown();
  return 0;
}
