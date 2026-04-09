#include <tf2/LinearMath/Quaternion.h>

#include <fastrack_demo/simple_planner.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fastrack_demo::PlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

namespace fastrack_demo
{

PlannerNode::PlannerNode()
  : Node("fastrack_planner_node")
{
  setupPlanner();
  service_ = this->create_service<fastrack_demo::srv::PlannedPath>("planned_path",
    std::bind(
      &PlannerNode::computePlannedPath, this, std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Simple Planner Started");
}

void PlannerNode::setupPlanner()
{
  if (planner_)
    return;

  this->declare_parameter<double>("path_length", 5.0);
  this->declare_parameter<double>("goal_x", 1.0);
  this->declare_parameter<double>("goal_y", -0.5);

  double path_length = this->get_parameter("path_length").as_double();
  double goal_x = this->get_parameter("goal_x").as_double();
  double goal_y = this->get_parameter("goal_y").as_double();
  planner_ = std::make_shared<Planner>(path_length, goal_x, goal_y);
}

void PlannerNode::computePlannedPath(
  std::shared_ptr<fastrack_demo::srv::PlannedPath::Request> request,
  std::shared_ptr<fastrack_demo::srv::PlannedPath::Response> response)
{
  std::lock_guard<std::mutex> lock(mutex_);
  double x = request->robot_x;
  double y = request->robot_y;
  response->path = planner_->generatePlan(x, y);
}

Planner::Planner(double path_length, double goal_x, double goal_y)
  : path_length_(path_length)
  , goal_x_(goal_x)
  , goal_y_(goal_y)
{
}

bool Planner::atGoal(double x, double y)
{
  (void)y;
  double dx = goal_x_ - x;
  // double dy = goal_y_ - y;

  // double distance_error = std::sqrt(dx*dx + dy*dy);
  double distance_error = dx;
  return distance_error < 0.05;  // arbitrary error
}

nav_msgs::msg::Path Planner::generatePlan(double current_x, double current_y)
{
  Path path;
  path.header.frame_id = "map";

  if (atGoal(current_x, current_y)) {
    return path;
  }

  // create a straight line path
  double step_size = 0.05;
  int num_points = static_cast<int>(path_length_ / step_size);

  for (int i = 0; i <= num_points; ++i) {
    double x = current_x + (i * step_size);
    double y = current_y;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;

    // straight line along X axis
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    // orientation facing forward
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pose.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(pose);
    if (atGoal(x, y)) {
      break;
    }
  }
  return path;
}
}  // namespace fastrack_demo
