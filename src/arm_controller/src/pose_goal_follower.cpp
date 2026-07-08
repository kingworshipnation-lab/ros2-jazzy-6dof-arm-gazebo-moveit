#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::PoseStamped normalizePose(
  const geometry_msgs::msg::PoseStamped & input,
  const std::string & default_frame)
{
  auto pose = input;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = default_frame;
  }

  const auto & q = pose.pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm < 1e-9) {
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
  } else {
    pose.pose.orientation.x /= norm;
    pose.pose.orientation.y /= norm;
    pose.pose.orientation.z /= norm;
    pose.pose.orientation.w /= norm;
  }
  return pose;
}

double positionDistance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  const double dz = a.pose.position.z - b.pose.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double orientationDistance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  const auto & qa = a.pose.orientation;
  const auto & qb = b.pose.orientation;
  const double dot = std::abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w);
  return 2.0 * std::acos(std::min(1.0, std::max(-1.0, dot)));
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

void scaleTrajectoryTime(
  moveit::planning_interface::MoveGroupInterface::Plan & plan,
  double scale)
{
  if (scale <= 1.0) {
    return;
  }

  for (auto & point : plan.trajectory.joint_trajectory.points) {
    setTimeFromStart(point, timeFromStart(point) * scale);
  }
}

double clampAngle(double radians)
{
  return std::max(0.0, std::min(kPi, radians));
}
}  // namespace

class PoseGoalFollower : public rclcpp::Node
{
public:
  explicit PoseGoalFollower(const rclcpp::NodeOptions & options)
  : Node("pose_goal_follower", options)
  {
  }

  void initialize()
  {
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    end_effector_link_ = declare_parameter<std::string>("end_effector_link", "tool_link");
    target_topic_ = declare_parameter<std::string>("target_topic", "/arm/target_pose");
    planning_time_ = declare_parameter<double>("planning_time", 5.0);
    min_plan_interval_ = declare_parameter<double>("min_plan_interval", 1.0);
    position_ignore_threshold_ = declare_parameter<double>("position_ignore_threshold", 0.005);
    orientation_ignore_threshold_ = declare_parameter<double>("orientation_ignore_threshold", 0.02);
    velocity_scaling_ = declare_parameter<double>("max_velocity_scaling_factor", 0.15);
    acceleration_scaling_ = declare_parameter<double>("max_acceleration_scaling_factor", 0.15);
    trajectory_time_scale_ = declare_parameter<double>("trajectory_time_scale", 1.5);
    position_goal_tolerance_ = declare_parameter<double>("position_goal_tolerance", 0.02);
    orientation_goal_tolerance_ = declare_parameter<double>("orientation_goal_tolerance", 3.14159);
    position_only_mode_ = declare_parameter<bool>("position_only_mode", true);
    allow_position_only_fallback_ = declare_parameter<bool>("allow_position_only_fallback", true);

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),
      planning_group_);
    planning_frame_ = move_group_->getPlanningFrame();
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPoseReferenceFrame(planning_frame_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->setGoalPositionTolerance(position_goal_tolerance_);
    move_group_->setGoalOrientationTolerance(orientation_goal_tolerance_);
    move_group_->startStateMonitor();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      target_topic_,
      rclcpp::QoS(10),
      std::bind(&PoseGoalFollower::targetCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Ready: subscribe %s, planning group '%s', tip '%s', planning frame '%s'.",
      target_topic_.c_str(),
      planning_group_.c_str(),
      end_effector_link_.c_str(),
      planning_frame_.c_str());
  }

private:
  void targetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto target = normalizePose(*msg, planning_frame_);
    const auto now = get_clock()->now();

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (busy_) {
        pending_target_ = target;
        RCLCPP_WARN(
          get_logger(),
          "Planning/execution is busy; queued the latest target and replaced any older pending target.");
        return;
      }
      busy_ = true;
    }

    if (last_plan_time_.nanoseconds() > 0 &&
      (now - last_plan_time_).seconds() < min_plan_interval_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring target: limited to %.2f Hz.",
        1.0 / std::max(1e-6, min_plan_interval_));
      finishCycleAndProcessPending();
      return;
    }

    if (last_target_) {
      const double dp = positionDistance(target, *last_target_);
      const double da = orientationDistance(target, *last_target_);
      if (dp < position_ignore_threshold_ && da < orientation_ignore_threshold_) {
        RCLCPP_INFO(
          get_logger(),
          "Ignoring tiny target change: position %.4f m, orientation %.4f rad.",
          dp,
          da);
        finishCycleAndProcessPending();
        return;
      }
    }

    last_plan_time_ = now;
    last_target_ = target;

    logPoseError("before", target);
    RCLCPP_INFO(
      get_logger(),
      "Planning to target position %.3f %.3f %.3f in frame '%s'.",
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z,
      target.header.frame_id.c_str());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode result;
    if (position_only_mode_) {
      RCLCPP_INFO(get_logger(), "Using position-only target mode for stable Gazebo tracking.");
      move_group_->setPositionTarget(
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z,
        end_effector_link_);
      result = move_group_->plan(plan);
    } else {
      move_group_->setPoseTarget(target, end_effector_link_);
      result = move_group_->plan(plan);
    }

    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      if (!position_only_mode_ && allow_position_only_fallback_) {
        RCLCPP_WARN(
          get_logger(),
          "Full pose planning failed; retrying position-only target with relaxed end-effector orientation.");
        move_group_->clearPoseTargets();
        move_group_->setPositionTarget(
          target.pose.position.x,
          target.pose.position.y,
          target.pose.position.z,
          end_effector_link_);
        result = move_group_->plan(plan);
      }

      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          get_logger(),
          "Planning failed. Target may be unreachable, in collision, outside joint limits, or over-constrained.");
        move_group_->clearPoseTargets();
        finishCycleAndProcessPending();
        return;
      }
    }

    makeTrajectoryTimesStrictlyIncreasing(plan);
    scaleTrajectoryTime(plan, trajectory_time_scale_);
    const auto execution_result = move_group_->execute(plan);
    if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Trajectory execution failed after a valid plan was generated.");
      move_group_->clearPoseTargets();
      finishCycleAndProcessPending();
      return;
    }

    logPoseError("after", target);
    RCLCPP_INFO(get_logger(), "Target pose execution completed.");
    move_group_->clearPoseTargets();
    finishCycleAndProcessPending();
  }

  void finishCycleAndProcessPending()
  {
    std::optional<geometry_msgs::msg::PoseStamped> next_target;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      next_target = pending_target_;
      pending_target_.reset();
      busy_ = false;
    }

    if (next_target) {
      auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>(*next_target);
      targetCallback(msg);
    }
  }

  void logPoseError(
    const std::string & label,
    const geometry_msgs::msg::PoseStamped & target)
  {
    const auto current = lookupCurrentPose(target.header.frame_id);
    if (!current) {
      RCLCPP_WARN(
        get_logger(),
        "Could not compute end-effector error %s execution: TF %s -> %s is unavailable.",
        label.c_str(),
        target.header.frame_id.c_str(),
        end_effector_link_.c_str());
      return;
    }

    const double dp = positionDistance(*current, target);
    const double da = clampAngle(orientationDistance(*current, target));
    RCLCPP_INFO(
      get_logger(),
      "End-effector error %s execution: position %.4f m, orientation %.4f rad.",
      label.c_str(),
      dp,
      da);
  }

  std::optional<geometry_msgs::msg::PoseStamped> lookupCurrentPose(const std::string & frame_id)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        frame_id,
        end_effector_link_,
        tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
      return std::nullopt;
    }
  }

  std::string planning_group_;
  std::string end_effector_link_;
  std::string target_topic_;
  std::string planning_frame_;
  double planning_time_{5.0};
  double min_plan_interval_{1.0};
  double position_ignore_threshold_{0.005};
  double orientation_ignore_threshold_{0.02};
  double velocity_scaling_{0.15};
  double acceleration_scaling_{0.15};
  double trajectory_time_scale_{1.5};
  double position_goal_tolerance_{0.02};
  double orientation_goal_tolerance_{3.14159};
  bool position_only_mode_{true};
  bool allow_position_only_fallback_{true};
  bool busy_{false};
  std::mutex state_mutex_;
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  std::optional<geometry_msgs::msg::PoseStamped> last_target_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_target_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<PoseGoalFollower>(options);
  node->initialize();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
