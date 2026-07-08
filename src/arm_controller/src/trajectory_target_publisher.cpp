#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

std::vector<std::array<double, 3>> makeLine(
  double cx,
  double cy,
  double cz,
  double length,
  int points)
{
  std::vector<std::array<double, 3>> targets;
  const int count = std::max(2, points);
  targets.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double t = (static_cast<double>(i) / static_cast<double>(count - 1)) - 0.5;
    targets.push_back({cx, cy + t * length, cz});
  }
  return targets;
}

std::vector<std::array<double, 3>> makeCircle(
  double cx,
  double cy,
  double cz,
  double radius,
  int points)
{
  std::vector<std::array<double, 3>> targets;
  const int count = std::max(4, points);
  targets.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(count);
    targets.push_back({cx, cy + radius * std::cos(theta), cz + radius * std::sin(theta)});
  }
  return targets;
}

std::vector<std::array<double, 3>> makeSquare(
  double cx,
  double cy,
  double cz,
  double size)
{
  const double half = 0.5 * size;
  return {
    {cx, cy - half, cz - half},
    {cx, cy + half, cz - half},
    {cx, cy + half, cz + half},
    {cx, cy - half, cz + half},
  };
}
}  // namespace

class TrajectoryTargetPublisher : public rclcpp::Node
{
public:
  TrajectoryTargetPublisher()
  : Node("trajectory_target_publisher")
  {
    topic_ = declare_parameter<std::string>("topic", "/arm/target_pose");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    mode_ = declare_parameter<std::string>("mode", "circle");
    rate_ = clamp(declare_parameter<double>("rate", 0.1), 0.01, 1.0);
    radius_ = clamp(declare_parameter<double>("radius", 0.05), 0.01, 0.10);
    line_length_ = clamp(declare_parameter<double>("line_length", 0.10), 0.02, 0.18);
    square_size_ = clamp(declare_parameter<double>("square_size", 0.10), 0.02, 0.18);
    center_x_ = clamp(declare_parameter<double>("center_x", 0.35), 0.20, 0.55);
    center_y_ = clamp(declare_parameter<double>("center_y", 0.0), -0.25, 0.25);
    center_z_ = clamp(declare_parameter<double>("center_z", 0.45), 0.25, 0.70);
    points_per_cycle_ = std::max(4, static_cast<int>(declare_parameter<int>("points_per_cycle", 8)));
    include_center_start_ = declare_parameter<bool>("include_center_start", true);

    publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic_, rclcpp::QoS(10));
    rebuildTargets();

    const auto period = std::chrono::duration<double>(1.0 / rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TrajectoryTargetPublisher::publishNextTarget, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing %s trajectory targets on %s at %.3f Hz around center %.3f %.3f %.3f.",
      mode_.c_str(),
      topic_.c_str(),
      rate_,
      center_x_,
      center_y_,
      center_z_);

    if (rate_ > 0.2) {
      RCLCPP_WARN(
        get_logger(),
        "Requested rate %.3f Hz may be faster than MoveIt planning/execution throughput. "
        "Use 0.05-0.10 Hz if every point must execute cleanly.",
        rate_);
    }
  }

private:
  void rebuildTargets()
  {
    if (mode_ == "line") {
      targets_ = makeLine(center_x_, center_y_, center_z_, line_length_, points_per_cycle_);
    } else if (mode_ == "square") {
      targets_ = makeSquare(center_x_, center_y_, center_z_, square_size_);
    } else {
      if (mode_ != "circle") {
        RCLCPP_WARN(get_logger(), "Unknown mode '%s'; falling back to circle.", mode_.c_str());
        mode_ = "circle";
      }
      targets_ = makeCircle(center_x_, center_y_, center_z_, radius_, points_per_cycle_);
    }

    if (include_center_start_) {
      targets_.insert(targets_.begin(), {center_x_, center_y_, center_z_});
    }
  }

  void publishNextTarget()
  {
    if (targets_.empty()) {
      return;
    }

    const auto & point = targets_[index_];
    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = now();
    target.header.frame_id = frame_id_;
    target.pose.position.x = point[0];
    target.pose.position.y = point[1];
    target.pose.position.z = point[2];
    target.pose.orientation.x = 0.0;
    target.pose.orientation.y = 0.0;
    target.pose.orientation.z = 0.0;
    target.pose.orientation.w = 1.0;

    publisher_->publish(target);
    RCLCPP_INFO(
      get_logger(),
      "Published target %zu/%zu: x %.3f, y %.3f, z %.3f.",
      index_ + 1,
      targets_.size(),
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z);

    index_ = (index_ + 1) % targets_.size();
  }

  std::string topic_;
  std::string frame_id_;
  std::string mode_;
  double rate_{0.1};
  double radius_{0.05};
  double line_length_{0.10};
  double square_size_{0.10};
  double center_x_{0.35};
  double center_y_{0.0};
  double center_z_{0.45};
  int points_per_cycle_{8};
  bool include_center_start_{true};
  std::size_t index_{0};
  std::vector<std::array<double, 3>> targets_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTargetPublisher>());
  rclcpp::shutdown();
  return 0;
}
