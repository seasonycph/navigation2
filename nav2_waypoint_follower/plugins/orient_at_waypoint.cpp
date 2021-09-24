#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <exception>
#include <functional>

#include "nav2_util/node_utils.hpp"
#include "nav2_waypoint_follower/plugins/orient_at_waypoint.hpp"

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
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        const std::string &plugin_name)
    {
        auto node = parent.lock();
        if (!node)
        {
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
        if (waypoint_pause_duration_ == 0)
        {
            is_enabled_ = false;
            RCLCPP_INFO(
                logger_,
                "Waypoint pause duration is set to zero, disabling task executor plugin.");
        }
        else if (!is_enabled_)
        {
            RCLCPP_INFO(
                logger_, "Waypoint task executor plugin is disabled.");
        }
        callback_group_ = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, node->get_node_base_interface());
        orientation_client_ = rclcpp_action::create_client<nav2_msgs::action::Orientation>(
            node->get_node_base_interface(),
            node->get_node_graph_interface(),
            node->get_node_logging_interface(),
            node->get_node_waitables_interface(),
            "lewis/orientation_manager", callback_group_);
    }

    bool OrientAtWaypoint::processAtWaypoint(
        const geometry_msgs::msg::PoseStamped & /*curr_pose*/, const int &curr_waypoint_index)
    {
        if (!is_enabled_)
        {
            return true;
        }
        using namespace std::placeholders;
        //auto node = parent.lock();
        result_received_ = false;
        //send goal
        auto goal_msg = nav2_msgs::action::Orientation::Goal();
        goal_msg.goal_pose.z = 0.999990380295387;
        goal_msg.goal_pose.w = 0.00438626454830838;
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::Orientation>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&OrientAtWaypoint::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&OrientAtWaypoint::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&OrientAtWaypoint::result_callback, this, _1);
        // todo, check for goal response feedback
        auto future_goal = orientation_client_->async_send_goal(goal_msg, send_goal_options);

        while (!result_received_)
        {
            callback_group_executor_.spin_some();
            rclcpp::sleep_for(std::chrono::milliseconds(waypoint_pause_duration_));
            RCLCPP_INFO(
                logger_, "Task incomplete at %i'th waypoint, sleeping for %i milliseconds",
                curr_waypoint_index,
                waypoint_pause_duration_);
        }

        return true;
        //extra params -> last node
        //action server, call rotation (orientation) manager with (current position, end position)
    }

    void OrientAtWaypoint::goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Orientation>::SharedPtr &future)
    {
        RCLCPP_INFO(logger_, "ChaCHING");
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_INFO(logger_, "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(logger_, "Goal accepted by server, waiting for result");
        }
    }

    void OrientAtWaypoint::feedback_callback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::Orientation>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::Orientation::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Next number in sequence received: " << feedback->distance;
        RCLCPP_INFO(logger_, ss.str().c_str());
    }

    void OrientAtWaypoint::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Orientation>::WrappedResult &result)
    {
        result_received_ = true;
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(logger_, "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(logger_, "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(logger_, "Unknown result code");
            return;
        }
        std::stringstream ss;
        ss << "Result received: ";
        // for (auto number : result.result->sequence)
        // {
        //     ss << number << " ";
        // }
        RCLCPP_INFO(logger_, ss.str().c_str());
        rclcpp::shutdown();
    }
} // namespace nav2_waypoint_follower
PLUGINLIB_EXPORT_CLASS(
    nav2_waypoint_follower::OrientAtWaypoint,
    nav2_core::WaypointTaskExecutor)
