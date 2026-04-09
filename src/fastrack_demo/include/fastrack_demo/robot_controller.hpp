#include <fastrack_demo/dubins_car.hpp>
#include <fastrack_demo/srv/tracked_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;

using fastrack_demo::srv::TrackedPose;

namespace fastrack_demo
{

class RobotController : public rclcpp::Node
{
  public:
  RobotController();

  private:
  void setupParameters();
  void odomCallback(Odometry::SharedPtr const& msg);
  void trackedPathResponse(Pose::SharedPtr const& pose);
  void publishControl(dubins_car::TrackerControl const& c);
  void controlLoop();

  // ROS2 components
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // data
  Path::SharedPtr curr_planned_path_;
  Odometry::SharedPtr current_odom_;
  std::mutex mutex_;

  rclcpp::Client<TrackedPose>::SharedPtr tracked_pose_client_;
  dubins_car::DubinsCarParameters params_;
  dubins_car::DubinsCarHybridTrackingController controller_;
};

}  // namespace fastrack_demo
