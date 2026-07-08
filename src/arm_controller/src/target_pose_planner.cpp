#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
geometry_msgs::msg::PoseStamped normalizePose(const geometry_msgs::msg::PoseStamped & input)
{
  auto pose = input;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = "world";
  }

  const auto & q = pose.pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1e-9) {
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
  } else {
    pose.pose.orientation.x /= norm;
    pose.pose.orientation.y /= norm;
    pose.pose.orientation.z /= norm;
    pose.pose.orientation.w /= norm;
  }
  return pose;
}

void setTimeFromStart(trajectory_msgs::msg::JointTrajectoryPoint & point, double seconds)
{
  const auto whole_seconds = static_cast<int32_t>(std::floor(seconds));
  point.time_from_start.sec = whole_seconds;
  point.time_from_start.nanosec = static_cast<uint32_t>((seconds - whole_seconds) * 1e9);
}

double timeFromStart(const trajectory_msgs::msg::JointTrajectoryPoint & point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void makeTrajectoryTimesStrictlyIncreasing(moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  auto & points = plan.trajectory.joint_trajectory.points;
  double previous_time = -0.5;
  for (auto & point : points) {
    const double current_time = timeFromStart(point);
    if (current_time <= previous_time) {
      setTimeFromStart(point, previous_time + 0.5);
    }
    previous_time = timeFromStart(point);
  }
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("target_pose_planner", options);

  auto planning_group = node->declare_parameter<std::string>("planning_group", "arm");
  auto end_effector_link = node->declare_parameter<std::string>("end_effector_link", "tool_link");
  auto pose_reference_frame = node->declare_parameter<std::string>("pose_reference_frame", "world");
  auto planning_time = node->declare_parameter<double>("planning_time", 5.0);
  auto target_topic = node->declare_parameter<std::string>("target_topic", "/arm/target_pose");
  auto velocity_scaling = node->declare_parameter<double>("max_velocity_scaling_factor", 0.3);
  auto acceleration_scaling = node->declare_parameter<double>("max_acceleration_scaling_factor", 0.3);

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  move_group.setEndEffectorLink(end_effector_link);
  move_group.setPoseReferenceFrame(pose_reference_frame);
  move_group.setPlanningTime(planning_time);
  move_group.setMaxVelocityScalingFactor(velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(acceleration_scaling);
  move_group.startStateMonitor();

  RCLCPP_INFO(
    node->get_logger(),
    "Ready: publish geometry_msgs/msg/PoseStamped on %s for group '%s', tip '%s'.",
    target_topic.c_str(),
    planning_group.c_str(),
    end_effector_link.c_str());

  auto subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_topic,
    rclcpp::QoS(10),
    [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      const auto target = normalizePose(*msg);
      RCLCPP_INFO(
        node->get_logger(),
        "Planning to %.3f %.3f %.3f in frame '%s'.",
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z,
        target.header.frame_id.c_str());

      move_group.setPoseTarget(target, end_effector_link);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto result = move_group.plan(plan);
      const bool planned = result == moveit::core::MoveItErrorCode::SUCCESS;
      if (!planned) {
        RCLCPP_ERROR(node->get_logger(), "MoveIt could not find a valid plan for the requested pose.");
        move_group.clearPoseTargets();
        return;
      }

      makeTrajectoryTimesStrictlyIncreasing(plan);
      const auto execution_result = move_group.execute(plan);
      if (execution_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Trajectory execution completed.");
      } else {
        RCLCPP_ERROR(node->get_logger(), "Trajectory execution failed.");
      }
      move_group.clearPoseTargets();
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
