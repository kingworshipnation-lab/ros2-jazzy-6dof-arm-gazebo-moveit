#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("send_pose_goal");

  const auto topic = node->declare_parameter<std::string>("topic", "/arm/target_pose");
  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = node->declare_parameter<std::string>("frame_id", "world");
  target.pose.position.x = node->declare_parameter<double>("x", 0.0);
  target.pose.position.y = node->declare_parameter<double>("y", 0.0);
  target.pose.position.z = node->declare_parameter<double>("z", 1.82);
  target.pose.orientation.x = node->declare_parameter<double>("qx", 0.0);
  target.pose.orientation.y = node->declare_parameter<double>("qy", 0.0);
  target.pose.orientation.z = node->declare_parameter<double>("qz", 0.0);
  target.pose.orientation.w = node->declare_parameter<double>("qw", 1.0);

  auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
  rclcpp::WallRate wait_rate(20.0);
  for (int i = 0; rclcpp::ok() && publisher->get_subscription_count() == 0 && i < 60; ++i) {
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  target.header.stamp = node->now();
  publisher->publish(target);
  rclcpp::spin_some(node);

  RCLCPP_INFO(
    node->get_logger(),
    "Published pose goal on %s: %.3f %.3f %.3f.",
    topic.c_str(),
    target.pose.position.x,
    target.pose.position.y,
    target.pose.position.z);

  rclcpp::shutdown();
  return 0;
}
