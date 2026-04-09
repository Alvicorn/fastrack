#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <fastrack_demo/simple_tracker.hpp>
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fastrack_demo::TrackerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

namespace fastrack_demo
{
TrackerNode::TrackerNode()
  : Node("fastrack_tracker_node")
{
  setupTracker();
  planned_path_client_ = create_client<PlannedPath>("planned_path");
  service_ = this->create_service<fastrack_demo::srv::TrackedPose>(
  "tracked_pose",
  [this](const std::shared_ptr<TrackedPose::Request> request,
         std::shared_ptr<TrackedPose::Response> response)
  {
      computeTrackedPath(request, response);
  });

  RCLCPP_INFO(this->get_logger(), "Simple FaSTrack Tracker Started");
}

void TrackerNode::setupTracker()
{
  if (tracker_)
    return;
  this->declare_parameter<double>("min_lookahead", 0.05);
  this->declare_parameter<double>("max_lookahead", 0.5);
  double min_lookahead = this->get_parameter("min_lookahead").as_double();
  double max_lookahead = this->get_parameter("max_lookahead").as_double();
  tracker_ = std::make_shared<Tracker>(min_lookahead, max_lookahead);
}

void TrackerNode::computeTrackedPath(
  std::shared_ptr<TrackedPose::Request> const& request,
  std::shared_ptr<TrackedPose::Response> const& response)
{
  std::scoped_lock lock(mutex_);

  auto planned_request = std::make_shared<PlannedPath::Request>();

  planned_request->robot_x = request->robot_x;
  planned_request->robot_y = request->robot_y;

  if (!planned_path_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(this->get_logger(), "Planned path service not available");
    return;
  }

  auto future_result = planned_path_client_->async_send_request(planned_request,
    [this](rclcpp::Client<PlannedPath>::SharedFuture result_future) {
      auto result = result_future.get();
      Path::SharedPtr path_ptr = std::make_shared<Path>(result->path);
      tracker_->setPath(path_ptr);
    });

  double robot_x = request->robot_x;
  double robot_y = request->robot_y;
  geometry_msgs::msg::Pose lookahead_pose;
  if (!tracker_->getLookaheadPoint(robot_x, robot_y, lookahead_pose)) {
    return;
  }

  response->pose = lookahead_pose;
}

Tracker::Tracker(double min_lookahead, double max_lookahead)
  : min_lookahead_(min_lookahead)
  , max_lookahead_(max_lookahead)
{
}

void Tracker::setPath(Path::SharedPtr const& path)
{
  std::scoped_lock lock(mtx_);
  path_ = path;
}

double Tracker::lookaheadDistance(double x, double y)
{
  if (!path_ || path_->poses.empty()) {
    return min_lookahead_;
  }

  auto const& goal_pose = path_->poses.back().pose;
  double dx = goal_pose.position.x - x;
  double dy = goal_pose.position.y - y;
  double dist = std::sqrt((dx * dx) + (dy * dy));

  return std::clamp(dist * 0.5, min_lookahead_, max_lookahead_);
}

bool Tracker::getLookaheadPoint(double robot_x, double robot_y, Pose& lookahead_pose)
{
  std::scoped_lock lock(mtx_);
  if (!path_ || path_->poses.empty()) {
    return false;
  }

  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path_->poses.size(); i++) {
    double dx = path_->poses[i].pose.position.x - robot_x;
    double dy = path_->poses[i].pose.position.y - robot_y;
    double dist = std::sqrt((dx * dx) + (dy * dy));

    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  double accumulated_dist = 0.0;
  double lookahead = lookaheadDistance(robot_x, robot_y);
  size_t lookahead_idx = closest_idx;

  for (size_t i = closest_idx + 1; i < path_->poses.size(); ++i) {
    double dx = path_->poses[i].pose.position.x - path_->poses[i - 1].pose.position.x;
    double dy = path_->poses[i].pose.position.y - path_->poses[i - 1].pose.position.y;

    accumulated_dist += std::sqrt((dx * dx) + (dy * dy));

    if (accumulated_dist >= lookahead) {
      lookahead_idx = i;
      break;
    }
  }

  lookahead_pose = path_->poses[lookahead_idx].pose;
  return true;
}

}  // namespace fastrack_demo
