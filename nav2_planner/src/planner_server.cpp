// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2019 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav2_planner/planner_server.hpp"

using namespace std::chrono_literals;

namespace nav2_planner
{

  PlannerServer::PlannerServer()
      : nav2_util::LifecycleNode("nav2_planner", "", true),
        gp_loader_("nav2_core", "nav2_core::GlobalPlanner"),
        default_ids_{"GridBased"},
        default_types_{"nav2_navfn_planner/NavfnPlanner"},
        costmap_(nullptr)
  {
    RCLCPP_INFO(get_logger(), "Creating");

    // Declare this node's parameters
    declare_parameter("planner_plugins", default_ids_);
    declare_parameter("expected_planner_frequency", 1.0);

    get_parameter("planner_plugins", planner_ids_);
    if (planner_ids_ == default_ids_)
    {
      for (size_t i = 0; i < default_ids_.size(); ++i)
      {
        declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
      }
    }

    // Setup the global costmap
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "global_costmap", std::string{get_namespace()}, "global_costmap");
    //current_path_ =
    // Launch a thread to run the costmap node
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
  }

  PlannerServer::~PlannerServer()
  {
    planners_.clear();
    costmap_thread_.reset();
  }

  nav2_util::CallbackReturn
  PlannerServer::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    costmap_ros_->on_configure(state);
    costmap_ = costmap_ros_->getCostmap();

    RCLCPP_DEBUG(
        get_logger(), "Costmap size: %d,%d",
        costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

    tf_ = costmap_ros_->getTfBuffer();

    planner_types_.resize(planner_ids_.size());

    auto node = shared_from_this();

    for (size_t i = 0; i != planner_ids_.size(); i++)
    {
      try
      {
        planner_types_[i] = nav2_util::get_plugin_type_param(
            node, planner_ids_[i]);
        nav2_core::GlobalPlanner::Ptr planner =
            gp_loader_.createUniqueInstance(planner_types_[i]);
        RCLCPP_INFO(
            get_logger(), "Created global planner plugin %s of type %s",
            planner_ids_[i].c_str(), planner_types_[i].c_str());
        planner->configure(node, planner_ids_[i], tf_, costmap_ros_);
        planners_.insert({planner_ids_[i], planner});
      }
      catch (const pluginlib::PluginlibException &ex)
      {
        RCLCPP_FATAL(
            get_logger(), "Failed to create global planner. Exception: %s",
            ex.what());
        return nav2_util::CallbackReturn::FAILURE;
      }
    }

    for (size_t i = 0; i != planner_ids_.size(); i++)
    {
      planner_ids_concat_ += planner_ids_[i] + std::string(" ");
    }

    RCLCPP_INFO(
        get_logger(),
        "Planner Server has %s planners available.", planner_ids_concat_.c_str());

    double expected_planner_frequency;
    get_parameter("expected_planner_frequency", expected_planner_frequency);
    if (expected_planner_frequency > 0)
    {
      max_planner_duration_ = 1 / expected_planner_frequency;
    }
    else
    {
      RCLCPP_WARN(
          get_logger(),
          "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
          " than 0.0 to turn on duration overrrun warning messages",
          expected_planner_frequency);
      max_planner_duration_ = 0.0;
    }

    // Initialize pubs & subs
    plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

    // Create the action servers for path planning to a pose and through poses
    action_server_pose_ = std::make_unique<ActionServerToPose>(
        rclcpp_node_,
        "compute_path_to_pose",
        std::bind(&PlannerServer::computePlan, this));

    action_server_poses_ = std::make_unique<ActionServerThroughPoses>(
        rclcpp_node_,
        "compute_path_through_poses",
        std::bind(&PlannerServer::computePlanThroughPoses, this));

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  PlannerServer::on_activate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Activating");

    plan_publisher_->on_activate();
    action_server_pose_->activate();
    action_server_poses_->activate();
    costmap_ros_->on_activate(state);

    PlannerMap::iterator it;
    for (it = planners_.begin(); it != planners_.end(); ++it)
    {
      it->second->activate();
    }

    // create bond connection
    createBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  PlannerServer::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");

    action_server_pose_->deactivate();
    action_server_poses_->deactivate();
    plan_publisher_->on_deactivate();
    costmap_ros_->on_deactivate(state);

    PlannerMap::iterator it;
    for (it = planners_.begin(); it != planners_.end(); ++it)
    {
      it->second->deactivate();
    }

    // destroy bond connection
    destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  PlannerServer::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");

    action_server_pose_.reset();
    action_server_poses_.reset();
    plan_publisher_.reset();
    tf_.reset();
    costmap_ros_->on_cleanup(state);

    PlannerMap::iterator it;
    for (it = planners_.begin(); it != planners_.end(); ++it)
    {
      it->second->cleanup();
    }
    planners_.clear();
    costmap_ = nullptr;

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  template <typename T>
  bool PlannerServer::isServerInactive(
      std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server)
  {
    if (action_server == nullptr || !action_server->is_server_active())
    {
      RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
      return true;
    }

    return false;
  }

  void PlannerServer::waitForCostmap()
  {
    // Don't compute a plan until costmap is valid (after clear costmap)
    rclcpp::Rate r(100);
    while (!costmap_ros_->isCurrent())
    {
      r.sleep();
    }
  }

  template <typename T>
  bool PlannerServer::isCancelRequested(
      std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server)
  {
    if (action_server->is_cancel_requested())
    {
      RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
      action_server->terminate_all();
      return true;
    }

    return false;
  }

  template <typename T>
  void PlannerServer::getPreemptedGoalIfRequested(
      std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
      typename std::shared_ptr<const typename T::Goal> goal)
  {
    if (action_server->is_preempt_requested())
    {
      goal = action_server->accept_pending_goal();
    }
  }

  template <typename T>
  bool PlannerServer::getStartPose(
      std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
      typename std::shared_ptr<const typename T::Goal> goal,
      geometry_msgs::msg::PoseStamped &start)
  {
    if (goal->use_start)
    {
      start = goal->start;
    }
    else if (!costmap_ros_->getRobotPose(start))
    {
      action_server->terminate_current();
      return false;
    }

    return true;
  }

  template <typename T>
  bool PlannerServer::transformPosesToGlobalFrame(
      std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
      geometry_msgs::msg::PoseStamped &curr_start,
      geometry_msgs::msg::PoseStamped &curr_goal)
  {
    if (!costmap_ros_->transformPoseToGlobalFrame(curr_start, curr_start) ||
        !costmap_ros_->transformPoseToGlobalFrame(curr_goal, curr_goal))
    {
      RCLCPP_WARN(
          get_logger(), "Could not transform the start or goal pose in the costmap frame");
      action_server->terminate_current();
      return false;
    }

    return true;
  }

  template <typename T>
  bool PlannerServer::validatePath(
      std::unique_ptr<nav2_util::SimpleActionServer<T>> &action_server,
      const geometry_msgs::msg::PoseStamped &goal,
      const nav_msgs::msg::Path &path,
      const std::string &planner_id)
  {
    if (path.poses.size() == 0)
    {
      RCLCPP_WARN(
          get_logger(), "Planning algorithm %s failed to generate a valid"
                        " path to (%.2f, %.2f)",
          planner_id.c_str(),
          goal.pose.position.x, goal.pose.position.y);
      action_server->terminate_current();
      RCLCPP_INFO(get_logger(), "Path invalid!");
      return false;
    }

    RCLCPP_DEBUG(
        get_logger(),
        "Found valid path of size %lu to (%.2f, %.2f)",
        path.poses.size(), goal.pose.position.x,
        goal.pose.position.y);
    RCLCPP_INFO(get_logger(), "Path valid!");
    return true;
  }

  void
  PlannerServer::computePlanThroughPoses()
  {
    auto start_time = steady_clock_.now();

    // Initialize the ComputePathToPose goal and result
    auto goal = action_server_poses_->get_current_goal();
    auto result = std::make_shared<ActionThroughPoses::Result>();
    nav_msgs::msg::Path concat_path;

    try
    {
      if (isServerInactive(action_server_poses_) || isCancelRequested(action_server_poses_))
      {
        return;
      }

      waitForCostmap();

      getPreemptedGoalIfRequested(action_server_poses_, goal);

      if (goal->goals.size() == 0)
      {
        RCLCPP_WARN(
            get_logger(),
            "Compute path through poses requested a plan with no viapoint poses, returning.");
        action_server_poses_->terminate_current();
      }

      // Use start pose if provided otherwise use current robot pose
      geometry_msgs::msg::PoseStamped start;
      if (!getStartPose(action_server_poses_, goal, start))
      {
        return;
      }

      // Get consecutive paths through these points
      std::vector<geometry_msgs::msg::PoseStamped>::iterator goal_iter;
      geometry_msgs::msg::PoseStamped curr_start, curr_goal;
      for (unsigned int i = 0; i != goal->goals.size(); i++)
      {
        // Get starting point
        if (i == 0)
        {
          curr_start = start;
        }
        else
        {
          curr_start = goal->goals[i - 1];
        }
        curr_goal = goal->goals[i];

        // Transform them into the global frame
        if (!transformPosesToGlobalFrame(action_server_poses_, curr_start, curr_goal))
        {
          return;
        }

        // Get plan from start -> goal
        nav_msgs::msg::Path curr_path = getPlan(curr_start, curr_goal, goal->planner_id);

        // check path for validity
        if (!validatePath(action_server_poses_, curr_goal, curr_path, goal->planner_id))
        {
          return;
        }

        // Concatenate paths together
        concat_path.poses.insert(
            concat_path.poses.end(), curr_path.poses.begin(), curr_path.poses.end());
        concat_path.header = curr_path.header;
      }

      // Publish the plan for visualization purposes
      result->path = concat_path;
      publishPlan(result->path);

      auto cycle_duration = steady_clock_.now() - start_time;
      result->planning_time = cycle_duration;

      if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_)
      {
        RCLCPP_WARN(
            get_logger(),
            "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
            1 / max_planner_duration_, 1 / cycle_duration.seconds());
      }

      action_server_poses_->succeeded_current(result);
    }
    catch (std::exception &ex)
    {
      RCLCPP_WARN(
          get_logger(),
          "%s plugin failed to plan through %li points with final goal (%.2f, %.2f): \"%s\"",
          goal->planner_id.c_str(), goal->goals.size(), goal->goals.back().pose.position.x,
          goal->goals.back().pose.position.y, ex.what());
      action_server_poses_->terminate_current();
    }
  }

  void
  PlannerServer::computePlan()
  {
    auto start_time = steady_clock_.now();
    RCLCPP_INFO(get_logger(), "Compute plan called.");
    // Initialize the ComputePathToPose goal and result
    auto goal = action_server_pose_->get_current_goal();
    auto result = std::make_shared<ActionToPose::Result>();

    try
    {
      if (isServerInactive(action_server_pose_) || isCancelRequested(action_server_pose_))
      {
        return;
      }

      waitForCostmap();

      getPreemptedGoalIfRequested(action_server_pose_, goal);

      // Use start pose if provided otherwise use current robot pose
      geometry_msgs::msg::PoseStamped start;
      if (!getStartPose(action_server_pose_, goal, start))
      {
        return;
      }

      // Transform them into the global frame
      geometry_msgs::msg::PoseStamped goal_pose = goal->goal;
      if (!transformPosesToGlobalFrame(action_server_pose_, start, goal_pose))
      {
        return;
      }

      result->path = getPlan(start, goal_pose, goal->planner_id);
      nav_msgs::msg::Path new_path_ = result->path;
      //todo - wrap in flag whether to use or not
      //use old path if the goal does not change
      int current_path_size = current_path_.poses.size();
      int new_path_size = result->path.poses.size();
      bool cull_points = false;
      if (current_path_size == 0)
      {
        RCLCPP_INFO(get_logger(), "Current path was empty, filling.");
        current_path_ = result->path;
        //set new cost array
        //todo - put this in a method
        double total_cost = 0;
        current_path_costs_.clear();
        for (long unsigned int i = 0; i < result->path.poses.size(); i++)
        {
          geometry_msgs::msg::PoseStamped ps = result->path.poses[i];
          unsigned int mx;
          unsigned int my;
          costmap_->worldToMap(ps.pose.position.x, ps.pose.position.y, mx, my);
          double cost = costmap_->getCost(mx, my);
          //RCLCPP_INFO(get_logger(), "Got cost %f for mx: %d and my: %d", cost, mx, my);
          total_cost = total_cost + cost;
          current_path_costs_.push_back(cost);
        }
        RCLCPP_INFO(
            get_logger(), "Total cost: %.2f", total_cost);
      }
      else if (current_path_size != 0 && new_path_size != 0)
      {
        RCLCPP_INFO(get_logger(), "Current path is populated, checking if new path is same goal.");
        geometry_msgs::msg::PoseStamped current_path_end = current_path_.poses[current_path_size - 1];
        geometry_msgs::msg::PoseStamped new_path_end = result->path.poses[new_path_size - 1];
        if ((current_path_end.pose.position.x == new_path_end.pose.position.x) && (current_path_end.pose.position.y == new_path_end.pose.position.y))
        {
          RCLCPP_INFO(get_logger(), "Path end goal does not differ, not replacing current path.");
          //TODO check similarity of current path and result path, they might have same goals but could differ vastly, in which case take the new path.
          cull_points = true;
        }
        else
        {
          current_path_ = result->path;
          RCLCPP_INFO(get_logger(), "Path end goal differs, replacing current path with new path.");
          //store total cost of path per pose point in array
          //reset array basically
          //todo - put in method
          double total_cost = 0;
          current_path_costs_.clear();
          for (long unsigned int i = 0; i < result->path.poses.size(); i++)
          {
            geometry_msgs::msg::PoseStamped ps = result->path.poses[i];
            unsigned int mx;
            unsigned int my;
            costmap_->worldToMap(ps.pose.position.x, ps.pose.position.y, mx, my);
            double cost = costmap_->getCost(mx, my);
            //RCLCPP_INFO(get_logger(), "Got cost %f for mx: %d and my: %d", cost, mx, my);
            total_cost = total_cost + cost;
            current_path_costs_.push_back(cost);
          }
          RCLCPP_INFO(
              get_logger(), "Total cost: %.2f", total_cost);
        }
      }

      //cull previous points - this implementation could be an issue if the path ahead loops close to current position, thn we cull too many points..
      //todo - this can be put into a method and called from where the cullpoints flag is set
      if (cull_points)
      {
        geometry_msgs::msg::PoseStamped new_path_start = result->path.poses[0];
        //calc distance to all points in current_path, find index where distance is shortest, cull points before that.
        int closest_index = -1;
        double distance = std::numeric_limits<double>::max();
        double static_point_x = new_path_start.pose.position.x;
        double static_point_y = new_path_start.pose.position.y;
        for (int i = 0; i < current_path_size; i++)
        {
          //distance from current position point to current_path[i] point
          double moving_point_x = current_path_.poses[i].pose.position.x;
          double moving_point_y = current_path_.poses[i].pose.position.y;
          double cur_dist = sqrt(pow(static_point_x - moving_point_x, 2) + pow(static_point_y - moving_point_y, 2));
          if (cur_dist < distance)
          {
            distance = cur_dist;
            closest_index = i;
          }
        }
        RCLCPP_INFO(
            get_logger(), "Closest pose index along path from current position: %d. Previous points will be culled", closest_index);
        std::vector<geometry_msgs::msg::PoseStamped> unculled_poses;
        unculled_poses = std::vector<geometry_msgs::msg::PoseStamped>(current_path_.poses.begin() + closest_index, current_path_.poses.end());
        current_path_.poses = unculled_poses;
        result->path = current_path_;

        std::vector<double> unculled_costs;
        unculled_costs = std::vector<double>(current_path_costs_.begin() + closest_index, current_path_costs_.end());
        current_path_costs_ = unculled_costs;
        //store array of cost values for each pose, cull values already past, calculate cost.
        //(old costmap, old plan for remaining distance)
        double total_cost_old = 0;
        for (const auto &value : unculled_costs)
        {
          total_cost_old = total_cost_old + value;
        }
        RCLCPP_INFO(
            get_logger(), "Total cost of current leftover path on previous cycle costs: %.2f", total_cost_old);
        // RCLCPP_INFO(
        //     get_logger(), "Total cost points: %ld", unculled_costs.size());
        //calculate cost of same path but with current cost values (new costmap, old plan for remaining distance)
        double total_cost_new = 0;
        for (const auto &ps : current_path_.poses)
        {
          unsigned int mx;
          unsigned int my;
          costmap_->worldToMap(ps.pose.position.x, ps.pose.position.y, mx, my);
          double cost = costmap_->getCost(mx, my);
          //RCLCPP_INFO(get_logger(), "Got cost %f for mx: %d and my: %d", cost, mx, my);
          total_cost_new = total_cost_new + cost;
        }
        RCLCPP_INFO(
            get_logger(), "Total cost of current leftover path on current cycle costs: %.2f", total_cost_new);
        // RCLCPP_INFO(
        //     get_logger(), "Total pose points: %ld", current_path_.poses.size());

        //percentage difference, set to 1s as to avoid divide by 0
        if (total_cost_old == 0)
        {
          total_cost_old = 1;
        }
        if (total_cost_new == 0)
        {
          total_cost_new = 1;
        }
        double difference = total_cost_new - total_cost_old;
        double ratio = difference / total_cost_old;
        double percentage = ratio * 100;
        RCLCPP_INFO(
            get_logger(), "Percentage increase: %.2f", percentage);
        RCLCPP_INFO(
            get_logger(), "Unit increase: %.2f", difference);
        //note - even if the path is the same and the underlying environment hasn't changed, costs can vary due to sensor noise -
        // as the inflation layer might oscillate a pixel or two. so when driving along the edge of two inflation layers, costs can increase/decrease.
        //if current cost values are higher than previous then wait / use_new path / fail
        //if using new path then clear current_path and current_costs
        if (percentage > 1 || percentage < -1) {
          if (percentage > 1)
            {
              RCLCPP_INFO(
                  get_logger(), "Cost has risen more than 1p, using new path instead");
            }
            else if (percentage < -1)
            {
              RCLCPP_INFO(
                  get_logger(), "Cost has fallen more than 1p, using new path instead");
            }
            
            if (percentage > 100 && obstruction_count_ <= 5)  //&& obstruction count < 5?
            {
              RCLCPP_INFO(
                  get_logger(), "Cost increase high, failing in order to wait.");
              obstruction_count_ = obstruction_count_ + 1;
              action_server_pose_->terminate_current();
              return;
            }
            else
            {
              result->path = new_path_;
              current_path_ = result->path;
              double total_cost = 0;
              current_path_costs_.clear();
              for (long unsigned int i = 0; i < result->path.poses.size(); i++)
              {
                geometry_msgs::msg::PoseStamped ps = result->path.poses[i];
                unsigned int mx;
                unsigned int my;
                costmap_->worldToMap(ps.pose.position.x, ps.pose.position.y, mx, my);
                double cost = costmap_->getCost(mx, my);
                //RCLCPP_INFO(get_logger(), "Got cost %f for mx: %d and my: %d", cost, mx, my);
                total_cost = total_cost + cost;
                current_path_costs_.push_back(cost);
              }
              RCLCPP_INFO(
                  get_logger(), "Total cost: %.2f", total_cost);
              obstruction_count_ = 0;
            }
        }
      }
      //todo - if cost is significantly lower, take new path.
      //todo - costmap locks
      //todo - also check if new path is empty to the same goal, meaning that the path became invalid. thus invalidate current_path
      //i.e. if path is empty then always set result.path to be empty

      RCLCPP_INFO(
          get_logger(), "Planned poses: %ld", result->path.poses.size());
      if (!validatePath(action_server_pose_, goal_pose, result->path, goal->planner_id))
      {
        return;
      }
      

      // Publish the plan for visualization purposes
      publishPlan(result->path);

      auto cycle_duration = steady_clock_.now() - start_time;
      result->planning_time = cycle_duration;

      if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_)
      {
        RCLCPP_WARN(
            get_logger(),
            "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
            1 / max_planner_duration_, 1 / cycle_duration.seconds());
      }

      action_server_pose_->succeeded_current(result);
    }
    catch (std::exception &ex)
    {
      RCLCPP_WARN(
          get_logger(), "%s plugin failed to plan calculation to (%.2f, %.2f): \"%s\"",
          goal->planner_id.c_str(), goal->goal.pose.position.x,
          goal->goal.pose.position.y, ex.what());
      action_server_pose_->terminate_current();
    }
  }

  nav_msgs::msg::Path
  PlannerServer::getPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal,
      const std::string &planner_id)
  {
    RCLCPP_DEBUG(
        get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
                      "(%.2f, %.2f).",
        start.pose.position.x, start.pose.position.y,
        goal.pose.position.x, goal.pose.position.y);
    RCLCPP_INFO(
        get_logger(), "Attempting to a find path from (%.2f, %.2f) to "
                      "(%.2f, %.2f).",
        start.pose.position.x, start.pose.position.y,
        goal.pose.position.x, goal.pose.position.y);
    if (planners_.find(planner_id) != planners_.end())
    {
      return planners_[planner_id]->createPlan(start, goal);
    }
    else
    {
      if (planners_.size() == 1 && planner_id.empty())
      {
        RCLCPP_WARN_ONCE(
            get_logger(), "No planners specified in action call. "
                          "Server will use only plugin %s in server."
                          " This warning will appear once.",
            planner_ids_concat_.c_str());
        return planners_[planners_.begin()->first]->createPlan(start, goal);
      }
      else
      {
        RCLCPP_ERROR(
            get_logger(), "planner %s is not a valid planner. "
                          "Planner names are: %s",
            planner_id.c_str(),
            planner_ids_concat_.c_str());
      }
    }

    return nav_msgs::msg::Path();
  }

  void
  PlannerServer::publishPlan(const nav_msgs::msg::Path &path)
  {
    auto msg = std::make_unique<nav_msgs::msg::Path>(path);
    if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0)
    {
      plan_publisher_->publish(std::move(msg));
    }
  }

} // namespace nav2_planner
