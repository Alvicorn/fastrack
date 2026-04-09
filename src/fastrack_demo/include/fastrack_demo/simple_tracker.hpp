#include <fastrack_demo/srv/planned_path.hpp>
#include <fastrack_demo/srv/tracked_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

using geometry_msgs::msg::Pose;
using nav_msgs::msg::Path;

using fastrack_demo::srv::TrackedPose;
using fastrack_demo::srv::PlannedPath;

namespace fastrack_demo
{
class Tracker
{
  public:
  Tracker(double min_lookahead, double max_lookahead);
  void setPath(Path::SharedPtr const& path);
  bool getLookaheadPoint(double robot_x, double robot_y, Pose& lookahead_pose);

  private:
  double lookaheadDistance(double x, double y);

  std::mutex mtx_;
  Path::SharedPtr path_;
  double min_lookahead_;
  double max_lookahead_;
};

class TrackerNode : public rclcpp::Node
{
  public:
  TrackerNode();

  void computeTrackedPath(std::shared_ptr<TrackedPose::Request> const& request,
    std::shared_ptr<TrackedPose::Response> const& response);

  private:
  void setupTracker();

  // ROS2 components
  rclcpp::Service<TrackedPose>::SharedPtr service_;
  rclcpp::Client<PlannedPath>::SharedPtr planned_path_client_;

  // data
  std::mutex mutex_;

  // backend service
  std::shared_ptr<Tracker> tracker_;
};

}  // namespace fastrack_demo
