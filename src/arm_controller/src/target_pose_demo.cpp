#include <array>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

struct NamedPose
{
  std::string name;
  std::array<double, 3> position;
  std::array<double, 4> orientation;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("target_pose_demo");

  const auto topic = node->declare_parameter<std::string>("topic", "/arm/target_pose");
  const auto frame_id = node->declare_parameter<std::string>("frame_id", "world");
  const auto period = node->declare_parameter<double>("period", 4.0);
  const auto repeat = node->declare_parameter<bool>("repeat", false);

  const std::vector<NamedPose> poses = {
    {"home", {0.00, 0.00, 1.82}, {0.0, 0.0, 0.0, 1.0}},
    {"front", {0.18, 0.00, 1.74}, {0.0, 0.0, 0.0, 1.0}},
    {"left", {0.00, 0.18, 1.74}, {0.0, 0.0, 0.0, 1.0}},
    {"upper", {0.00, 0.00, 1.70}, {0.0, 0.0, 0.0, 1.0}},
  };

  auto publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
  rclcpp::WallRate wait_rate(20.0);
  for (int i = 0; rclcpp::ok() && publisher->get_subscription_count() == 0 && i < 60; ++i) {
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  rclcpp::WallRate publish_rate(1.0 / period);
  do {
    for (const auto & pose : poses) {
      if (!rclcpp::ok()) {
        break;
      }
      geometry_msgs::msg::PoseStamped target;
      target.header.stamp = node->now();
      target.header.frame_id = frame_id;
      target.pose.position.x = pose.position[0];
      target.pose.position.y = pose.position[1];
      target.pose.position.z = pose.position[2];
      target.pose.orientation.x = pose.orientation[0];
      target.pose.orientation.y = pose.orientation[1];
      target.pose.orientation.z = pose.orientation[2];
      target.pose.orientation.w = pose.orientation[3];
      publisher->publish(target);
      RCLCPP_INFO(
        node->get_logger(),
        "Published %s pose goal: %.3f %.3f %.3f.",
        pose.name.c_str(),
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z);
      rclcpp::spin_some(node);
      publish_rate.sleep();
    }
  } while (rclcpp::ok() && repeat);

  rclcpp::shutdown();
  return 0;
}
