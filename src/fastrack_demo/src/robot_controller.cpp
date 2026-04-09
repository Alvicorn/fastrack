#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <fastrack_demo/robot_controller.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try{
    auto node = std::make_shared<fastrack_demo::RobotController>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

namespace fastrack_demo
{
RobotController::RobotController()
  : Node("robot_controller")
  , controller_(params_)
{
  setupParameters();

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, [this](Odometry::SharedPtr const msg) { odomCallback(msg); });
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  tracked_pose_client_ = create_client<fastrack_demo::srv::TrackedPose>("tracked_pose");
  timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this] { controlLoop(); });

  RCLCPP_INFO(this->get_logger(), "Simple FaSTrack Hybrid Tracking Controller Started");
}

void RobotController::setupParameters()
{
  this->declare_parameter<double>("min_v", 0.0);
  this->declare_parameter<double>("min_v_hat", 0.0);
  this->declare_parameter<double>("max_v", 0.5);
  this->declare_parameter<double>("max_v_hat", 0.5);
  this->declare_parameter<double>("max_omega", 1.5);
  this->declare_parameter<double>("max_omega_hat", 1.5);
  this->declare_parameter<double>("max_a", 0.5);
  this->declare_parameter<double>("max_alpha", 0.5);
  this->declare_parameter<double>("dt", 0.01);
  this->declare_parameter<double>("safety_threshold", 0.05);

  params_.min_v = this->get_parameter("min_v").as_double();
  params_.min_v_hat = this->get_parameter("min_v_hat").as_double();
  params_.max_v = this->get_parameter("max_v").as_double();
  params_.max_v_hat = this->get_parameter("max_v_hat").as_double();
  params_.max_omega = this->get_parameter("max_omega").as_double();
  params_.max_omega_hat = this->get_parameter("max_omega_hat").as_double();
  params_.max_a = this->get_parameter("max_a").as_double();
  params_.max_alpha = this->get_parameter("max_alpha").as_double();
  params_.dt = this->get_parameter("dt").as_double();
  params_.safety_threshold = this->get_parameter("safety_threshold").as_double();
}

void RobotController::odomCallback(Odometry::SharedPtr const& msg)
{
  std::scoped_lock lock(mutex_);
  current_odom_ = msg;
}

void RobotController::publishControl(dubins_car::TrackerControl const& c)
{
  if (!current_odom_)
    return;

  geometry_msgs::msg::Twist cmd;

  double v = current_odom_->twist.twist.linear.x;
  double omega = current_odom_->twist.twist.angular.z;

  v = std::clamp(c.a * params_.dt, params_.min_v, params_.max_v);
  omega = std::clamp(c.alpha * params_.dt, -params_.max_omega, params_.max_omega);

  cmd.linear.x = v;
  cmd.angular.z = omega;
  cmd_pub_->publish(cmd);
}

void RobotController::trackedPathResponse(Pose::SharedPtr const& pose)
{
  std::scoped_lock lock(mutex_);
  if (!current_odom_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odom data");
    return;  // Wait for data
  }

  double robot_x = current_odom_->pose.pose.position.x;
  double robot_y = current_odom_->pose.pose.position.y;
  double robot_v = current_odom_->twist.twist.linear.x;
  double robot_omega = current_odom_->twist.twist.angular.z;

  double roll;
  double pitch;
  double robot_yaw;
  double planned_yaw;

  // Get robot yaw
  tf2::Quaternion q_r;
  tf2::fromMsg(current_odom_->pose.pose.orientation, q_r);
  tf2::Matrix3x3(q_r).getRPY(roll, pitch, robot_yaw);

  // get planned yaw
  tf2::Quaternion q_p;
  tf2::fromMsg(pose->orientation, q_p);
  tf2::Matrix3x3(q_p).getRPY(roll, pitch, planned_yaw);

  dubins_car::PlannerState p = {pose->position.x, pose->position.y, planned_yaw};
  dubins_car::TrackerState s = {robot_x, robot_y, robot_yaw, robot_v, robot_omega};

  dubins_car::TrackerControl c = controller_.computeControl(s, p);

  publishControl(c);
}

void RobotController::controlLoop()
{
  std::scoped_lock lock(mutex_);

  if (!tracked_pose_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for service");
    return;
  }
  if (!current_odom_) {
    return;  // Wait for data
  }

  auto request = std::make_shared<fastrack_demo::srv::TrackedPose::Request>();

  // Get robot pose
  request->robot_x = current_odom_->pose.pose.position.x;
  request->robot_y = current_odom_->pose.pose.position.y;

  tracked_pose_client_->async_send_request(
    request, [this](rclcpp::Client<TrackedPose>::SharedFuture future_result) {
      auto result = future_result.get();
      Pose::SharedPtr pose_ptr = std::make_shared<Pose>(result->pose);
      trackedPathResponse(pose_ptr);
    });
}
}  // namespace fastrack_demo
