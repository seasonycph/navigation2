#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <exception>

#include "nav2_util/node_utils.hpp"
#include "nav2_waypoint_follower/plugins/wait_at_waypoint.hpp"

namespace nav2_waypoint_follower
{
OrientAtWaypoint::OrientAtWaypoint()
: waypoint_pause_duration_(0),
  is_enabled_(true)
{
}

OrientAtWaypoint::~OrientAtWaypoint()
{
}

void OrientAtWaypoint::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node in wait at waypoint plugin!"};
  }
  logger_ = node->get_logger();
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".waypoint_pause_duration",
    rclcpp::ParameterValue(0));
  nav2_util::declare_parameter_if_not_declared(
    node,
    plugin_name + ".enabled",
    rclcpp::ParameterValue(true));
  node->get_parameter(
    plugin_name + ".waypoint_pause_duration",
    waypoint_pause_duration_);
  node->get_parameter(
    plugin_name + ".enabled",
    is_enabled_);
  if (waypoint_pause_duration_ == 0) {
    is_enabled_ = false;
    RCLCPP_INFO(
      logger_,
      "Waypoint pause duration is set to zero, disabling task executor plugin.");
  } else if (!is_enabled_) {
    RCLCPP_INFO(
      logger_, "Waypoint task executor plugin is disabled.");
  }
}

bool OrientAtWaypoint::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & /*curr_pose*/, const int & curr_waypoint_index)
{
  if (!is_enabled_) {
    return true;
  }
  RCLCPP_INFO(
    logger_, "Arrived at %i'th waypoint, sleeping for %i milliseconds",
    curr_waypoint_index,
    waypoint_pause_duration_);
  rclcpp::sleep_for(std::chrono::milliseconds(waypoint_pause_duration_));
  return true;
}
}  // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
  nav2_waypoint_follower::OrientAtWaypoint,
  nav2_core::WaypointTaskExecutor)
