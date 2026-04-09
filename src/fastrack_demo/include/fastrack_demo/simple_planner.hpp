#include <tf2/LinearMath/Quaternion.h>

#include <fastrack_demo/srv/planned_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>

using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;

namespace fastrack_demo
{

class Planner
{
  public:
  Planner(double path_length, double goal_x, double goal_y);
  Path generatePlan(double current_x, double current_y);
  bool atGoal(double x, double y);

  private:
  double path_length_;
  double goal_x_;
  double goal_y_;
};

class PlannerNode : public rclcpp::Node
{
  public:
  PlannerNode();

  private:
  void setupPlanner();
  void computePlannedPath(std::shared_ptr<fastrack_demo::srv::PlannedPath::Request> request,
    std::shared_ptr<fastrack_demo::srv::PlannedPath::Response> response);

  // ROS2 components
  rclcpp::Service<fastrack_demo::srv::PlannedPath>::SharedPtr service_;

  // data
  std::mutex mutex_;

  // backend service
  std::shared_ptr<Planner> planner_;
};

}  // namespace fastrack_demo
