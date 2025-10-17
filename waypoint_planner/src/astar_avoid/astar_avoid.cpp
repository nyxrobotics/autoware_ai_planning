/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "waypoint_planner/astar_avoid/astar_avoid.h"
#include <cmath>
#include "amathutils_lib/amathutils.hpp"
#include "libwaypoint_follower/libwaypoint_follower.h"
#include "ros/console.h"

AstarAvoid::AstarAvoid() : nh_(), private_nh_("~")
{
  private_nh_.param<int>("safety_waypoints_size", safety_waypoints_size_, 100);
  private_nh_.param<double>("update_rate", update_rate_, 10.0);

  private_nh_.param<bool>("enable_avoidance", enable_avoidance_, false);
  private_nh_.param<bool>("use_back", use_back_, true);
  private_nh_.param<double>("avoid_waypoints_velocity", avoid_waypoints_velocity_, 10.0);
  private_nh_.param<int>("plan_start_index", plan_start_index_, 40);
  private_nh_.param<double>("replan_interval", replan_interval_, 0.5);
  private_nh_.param<int>("search_waypoints_size", search_waypoints_size_, 50);
  private_nh_.param<int>("search_waypoints_delta", search_waypoints_delta_, 2);
  private_nh_.param<int>("closest_search_size", closest_search_size_, 30);
  private_nh_.param<int>("stopline_ahead_num", stopline_ahead_num_, 1);
  private_nh_.param<double>("decel_limit", decel_limit_, 0.1);
  private_nh_.param<double>("accel_limit", accel_limit_, 0.5);
  private_nh_.param<double>("vel_min", vel_min_, 0.72);

  safety_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("safety_waypoints", 1, true);
  debug_pub_ = nh_.advertise<nav_msgs::Path>("debug", 1, true);
  costmap_sub_ = nh_.subscribe("costmap", 1, &AstarAvoid::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarAvoid::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &AstarAvoid::currentVelocityCallback, this);
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &AstarAvoid::baseWaypointsCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &AstarAvoid::closestIndexCallback, this);
  obstacle_waypoint_sub_ = nh_.subscribe("obstacle_waypoint", 1, &AstarAvoid::obstacleIndexCallback, this);

  rate_ = new ros::Rate(update_rate_);
}

void AstarAvoid::costmapCallback(const nav_msgs::OccupancyGrid& msg)
{
  costmap_ = msg;
  tf::poseMsgToTF(costmap_.info.origin, local2costmap_);
  costmap_initialized_ = true;
}

void AstarAvoid::currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_pose_global_ = msg;

  if (!enable_avoidance_)
  {
    current_pose_initialized_ = true;
  }
  else
  {
    current_pose_local_.pose = transformPose(
        current_pose_global_.pose, getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));
    current_pose_local_.header.frame_id = costmap_.header.frame_id;
    current_pose_local_.header.stamp = current_pose_global_.header.stamp;
    current_pose_initialized_ = true;
  }
}

void AstarAvoid::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  current_velocity_ = msg;
  current_velocity_initialized_ = true;
}

void AstarAvoid::baseWaypointsCallback(const autoware_msgs::Lane& msg)
{
  base_waypoints_ = msg;
  base_waypoints_initialized_ = true;
}

void AstarAvoid::closestIndexCallback(const std_msgs::Int32& msg)
{
  base_index_ = msg.data;
  base_index_initialized_ = true;
}

void AstarAvoid::obstacleIndexCallback(const std_msgs::Int32& msg)
{
  obstacle_index_ = msg.data;
}

void AstarAvoid::run()
{
  // check topics
  while (ros::ok())
  {
    ros::spinOnce();
    if (checkInitialized())
    {
      break;
    }
    ros::Duration(1.0).sleep();
  }

  // main loop
  ros::WallTime start_plan_time = ros::WallTime::now();
  ros::WallTime start_avoid_time = ros::WallTime::now();

  // reset obstacle index
  obstacle_index_ = -1;

  // relaying mode at startup
  astar_plan_status_ = AstarAvoid::AsterPlanStatus::IDLE;
  select_way_ = AstarAvoid::WayType::RELAY;
  is_move_ = false;
  found_obstacle_ = false;

  // Kick off a timer to publish final waypoints
  timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &AstarAvoid::publishWaypoints, this);

  while (ros::ok())
  {
    ros::spinOnce();

    // relay mode
    if (!enable_avoidance_)
    {
      rate_->sleep();
      continue;
    }

    // avoidance mode
    bool found_obstacle = (obstacle_index_ >= 0);
    int obstacle_index_distance = found_obstacle ? obstacle_index_ : std::numeric_limits<int>::max();
    bool request_aster_planning = found_obstacle && (obstacle_index_distance <= search_waypoints_size_);

    // Update avoiding index
    if (select_way_ == AstarAvoid::WayType::AVOID)
    {
      avoid_index_ = updateCurrentIndex(avoid_waypoints_, current_pose_global_.pose, avoid_index_);
    }
    else
    {
      avoid_index_ = -1;
    }

    // update state
    if (request_aster_planning && (ros::WallTime::now() - start_plan_time).toSec() > replan_interval_)
    {
      ROS_INFO("Start Plan: Request A* planning");
      if (planAvoidWaypoints(avoid_path_size_))
      {
        ROS_INFO("Plan -> Avoid, Found path");
        astar_plan_status_ = AstarAvoid::AsterPlanStatus::SUCCESS;
        select_way_ = AstarAvoid::WayType::AVOID;
        is_move_ = true;
        obstacle_index_ = -1;
        avoid_index_ = -1;
        avoid_index_ = updateCurrentIndex(avoid_waypoints_, current_pose_global_.pose, avoid_index_);
      }
      else
      {
        ROS_INFO("Plan -> Relay, Cannot find path");
        astar_plan_status_ = AstarAvoid::AsterPlanStatus::FAILURE;
        select_way_ = AstarAvoid::WayType::RELAY;
        is_move_ = false;
        avoid_index_ = -1;
      }
      start_plan_time = ros::WallTime::now();
    }
    // Check if goal reached
    if (select_way_ == AstarAvoid::WayType::AVOID && is_move_ == true)
    {
      if (avoid_index_ >= avoid_path_size_)
      {
        ROS_INFO("Avoid -> Relay, Reached goal");
        select_way_ = AstarAvoid::WayType::RELAY;
        is_move_ = true;
        avoid_index_ = -1;
      }
    }
    rate_->sleep();
  }
}

bool AstarAvoid::checkInitialized()
{
  // check for relay mode
  bool initialized = current_pose_initialized_ && base_index_initialized_ && base_waypoints_initialized_;

  if (!initialized)
  {
    if (!current_pose_initialized_)
    {
      ROS_WARN_THROTTLE(5, "Waiting for current_pose topic ...");
    }
    if (!base_index_initialized_)
    {
      ROS_WARN_THROTTLE(5, "Waiting for closest_waypoint topic ...");
    }
    if (!base_waypoints_initialized_)
    {
      ROS_WARN_THROTTLE(5, "Waiting for base_waypoints topic ...");
    }
  }

  // check for avoidance mode, additionally
  if (enable_avoidance_)
  {
    initialized = initialized && current_velocity_initialized_ && costmap_initialized_;

    if (!initialized)
    {
      if (!current_velocity_initialized_)
      {
        ROS_WARN_THROTTLE(5, "Waiting for current_velocity topic ...");
      }
      if (!costmap_initialized_)
      {
        ROS_WARN_THROTTLE(5, "Waiting for costmap topic ...");
      }
    }
  }

  return initialized;
}

bool AstarAvoid::planAvoidWaypoints(int& end_of_avoid_index)
{
  bool found_path = false;

  if (base_index_ < 0 || base_index_ >= static_cast<int>(base_waypoints_.waypoints.size()))
  {
    ROS_ERROR("Invalid base_index_ = %d", base_index_);
    return false;
  }

  auto it =
      base_index_ + obstacle_index_ + stopline_ahead_num_ + 1 > static_cast<int>(base_waypoints_.waypoints.size()) ?
          base_waypoints_.waypoints.end() :
          base_waypoints_.waypoints.begin() + base_index_ + obstacle_index_ + stopline_ahead_num_ + 1;
  if (std::find_if(base_waypoints_.waypoints.begin() + base_index_, it, [](const autoware_msgs::Waypoint& wp) {
        return wp.wpstate.stop_state == autoware_msgs::WaypointState::TYPE_STOPLINE;
      }) != it)
  {
    return false;
  }
  tf::Transform base2avoid;
  base2avoid.setOrigin(tf::Vector3(current_pose_global_.pose.position.x, current_pose_global_.pose.position.y,
                                   current_pose_global_.pose.position.z));
  base2avoid.setRotation(
      tf::Quaternion(current_pose_global_.pose.orientation.x, current_pose_global_.pose.orientation.y,
                     current_pose_global_.pose.orientation.z, current_pose_global_.pose.orientation.w));
  // update goal pose incrementally and execute A* search
  std::vector<geometry_msgs::Pose> goal_poses;
  std::vector<int> goal_indices;

  for (int i = closest_search_size_; i < static_cast<int>(search_waypoints_size_); i += search_waypoints_delta_)
  {
    // update goal index
    // Note: obstacle_index_ is supposed to be relative to base_waypoint_index_.
    //       However, obstacle_index_ is published by velocity_set node. The astar_avoid and velocity_set
    //       should be combined together to prevent this kind of inconsistency.
    int goal_index = base_index_ + obstacle_index_ + i;
    if (goal_index >= static_cast<int>(base_waypoints_.waypoints.size()))
    {
      break;
    }
    auto it2 = goal_index + stopline_ahead_num_ + 1 > static_cast<int>(base_waypoints_.waypoints.size()) ?
                   base_waypoints_.waypoints.end() :
                   base_waypoints_.waypoints.begin() + goal_index + stopline_ahead_num_ + 1;
    auto result = std::find_if(base_waypoints_.waypoints.begin() + goal_index - search_waypoints_delta_, it2,
                               [](const autoware_msgs::Waypoint& wp) {
                                 return wp.wpstate.stop_state == autoware_msgs::WaypointState::TYPE_STOPLINE;
                               });
    if (result != it2)
    {
      break;
    }
    // update goal pose
    goal_pose_global_ = base_waypoints_.waypoints[goal_index].pose;
    goal_pose_local_.header = costmap_.header;
    goal_pose_local_.pose = transformPose(goal_pose_global_.pose,
                                          getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));
    goal_poses.push_back(goal_pose_local_.pose);
    goal_indices.push_back(goal_index);
  }

  if (goal_poses.empty())
  {
    ROS_ERROR("Can't find goal. base_index_ = %d, obstacle_index_ = %d, stopline_ahead_num_ = %d, base_size = %lu",
              base_index_, obstacle_index_, stopline_ahead_num_, base_waypoints_.waypoints.size());
    return false;
  }

  // Get trasnform from base to avoid
  // initialize costmap for A* search
  astar_.initialize(costmap_);

  // execute astar search
  avoid_start_base_index_ = base_index_;
  found_path = astar_.makePlan(current_pose_local_.pose, goal_poses);

  if (found_path && !astar_.getPath().poses.empty())
  {
    debug_pub_.publish(astar_.getPath());
    // Get reached goal index
    avoid_finish_base_index_ = goal_indices.at(astar_.getGoalIndex());
    mergeAvoidWaypoints(astar_.getPath(), avoid_start_base_index_, avoid_finish_base_index_, end_of_avoid_index,
                        base2avoid);
    if (!avoid_waypoints_.waypoints.empty())
    {
      avoid_index_ = avoid_start_base_index_;
      ROS_INFO("Found GOAL at goal_index = %d, current_index = %d, path_size = %lu", avoid_finish_base_index_,
               avoid_start_base_index_, astar_.getPath().poses.size());
      astar_.reset();
      return true;
    }
    else
    {
      ROS_ERROR("Wrong path detected. goal_index = %d, waypoints size = %lu", avoid_finish_base_index_,
                avoid_waypoints_.waypoints.size());
      found_path = false;
    }
  }

  ROS_ERROR("Can't find goal. Retry. base_index_ = %d, obstacle_index_ = %d, stopline_ahead_num_ = %d, base_size = %lu",
            base_index_, obstacle_index_, stopline_ahead_num_, base_waypoints_.waypoints.size());
  astar_.reset();
  return false;
}

void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& path, const int start_index, const int goal_index,
                                     int& end_of_avoid_index)
{
  tf::Transform base2avoid = getTransform(base_waypoints_.header.frame_id, path.poses.front().header.frame_id);
  mergeAvoidWaypoints(path, start_index, goal_index, end_of_avoid_index, base2avoid);
}

void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& path, const int start_index, const int goal_index,
                                     int& end_of_avoid_index, tf::Transform base2avoid)
{
  int start_index_in = start_index;
  if (goal_index == -1 || goal_index < start_index)
  {
    return;
  }
  else if (start_index == -1)
  {
    start_index_in = 0;
  }

  // add waypoints before start index
  avoid_waypoints_.waypoints.clear();
  for (int i = 0; i < start_index_in; ++i)
  {
    avoid_waypoints_.waypoints.push_back(base_waypoints_.waypoints.at(i));
  }

  // set waypoints for avoiding
  if (use_back_)
  {
    int direction = 1;
    for (auto next_pose : path.poses)
    {
      autoware_msgs::Waypoint wp;
      wp.pose.header = base_waypoints_.header;
      // if the next_pose.pose.position.z value is smaller than 0, it means that the path is backward
      direction = (next_pose.pose.position.z < 0) ? -1 : 1;
      next_pose.pose.position.z = 0;
      wp.pose.pose = transformPose(next_pose.pose, base2avoid);
      wp.pose.pose.position.z = current_pose_global_.pose.position.z;         // height = const
      wp.twist.twist.linear.x = direction * avoid_waypoints_velocity_ / 3.6;  // velocity = const
      avoid_waypoints_.waypoints.push_back(wp);
    }
  }
  else
  {
    for (const auto& pose : path.poses)
    {
      autoware_msgs::Waypoint wp;
      wp.pose.header = base_waypoints_.header;
      wp.pose.pose = transformPose(pose.pose, base2avoid);
      wp.pose.pose.position.z = current_pose_global_.pose.position.z;  // height = const
      wp.twist.twist.linear.x = avoid_waypoints_velocity_ / 3.6;       // velocity = const
      avoid_waypoints_.waypoints.push_back(wp);
    }
  }

  // add waypoints after goal index
  for (int i = goal_index + 1; i < static_cast<int>(base_waypoints_.waypoints.size()); ++i)
  {
    avoid_waypoints_.waypoints.push_back(base_waypoints_.waypoints.at(i));
  }

  // smoothing connection point
  limitPathAccel(avoid_waypoints_, accel_limit_, decel_limit_, (vel_min_ / 3.6));
  // update index for merged waypoints
  end_of_avoid_index = start_index_in + path.poses.size() + 1;
}

void AstarAvoid::publishWaypoints(const ros::TimerEvent& e)
{
  // select waypoints
  autoware_msgs::Lane current_waypoints;
  int current_index;
  if (select_way_ == AstarAvoid::WayType::AVOID)
  {
    current_waypoints = avoid_waypoints_;
    current_index = avoid_index_;
  }
  else
  {
    current_waypoints = base_waypoints_;
    current_index = base_index_;
  }
  if (current_index < 0 || current_index >= static_cast<int>(current_waypoints.waypoints.size()))
  {
    ROS_WARN("Invalid index: %d (between 0 and %d)", current_index,
             static_cast<int>(current_waypoints.waypoints.size()));
    astar_plan_status_ = AstarAvoid::AsterPlanStatus::FAILURE;
    select_way_ = AstarAvoid::WayType::RELAY;
    is_move_ = false;
    avoid_index_ = -1;
    return;
  }

  // Create local path starting at closest global waypoint
  autoware_msgs::Lane local_waypoints;
  local_waypoints.header = current_waypoints.header;
  local_waypoints.increment = current_waypoints.increment;
  for (int i = current_index;
       i < current_index + safety_waypoints_size_ && i < static_cast<int>(current_waypoints.waypoints.size()); ++i)
  {
    local_waypoints.waypoints.push_back(current_waypoints.waypoints[i]);
  }

  if (!local_waypoints.waypoints.empty())
  {
    safety_waypoints_pub_.publish(local_waypoints);
  }
  else
  {
    ROS_WARN("No waypoints to publish");
    astar_plan_status_ = AstarAvoid::AsterPlanStatus::FAILURE;
    select_way_ = AstarAvoid::WayType::RELAY;
    is_move_ = false;
    avoid_index_ = -1;
  }
}

tf::Transform AstarAvoid::getTransform(const std::string& from, const std::string& to)
{
  tf::StampedTransform stf;
  if (from.empty() || to.empty())
  {
    ROS_ERROR("Invalid frame_id: form = %s, to = %s", from.c_str(), to.c_str());
    return stf;
  }
  try
  {
    tf_listener_.lookupTransform(from, to, ros::Time(0), stf);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("Failed to get transform from %s to %s", from.c_str(), to.c_str());
  }
  return stf;
}

void AstarAvoid::limitPathAccel(autoware_msgs::Lane& path, double accel, double decel, double vel_min)
{
  if (path.waypoints.size() < 2)
  {
    return;
  }

  // Limit acceleration
  double prev_vel = path.waypoints.front().twist.twist.linear.x;
  for (size_t i = 1; i < path.waypoints.size(); ++i)
  {
    double dist =
        amathutils::find_distance(path.waypoints[i - 1].pose.pose.position, path.waypoints[i].pose.pose.position);

    double travel_time = 1.0;
    if (fabs(prev_vel) > 0.0001)
    {
      travel_time = dist / fabs(prev_vel);
    }

    // Calculate velocity limits based on acceleration
    double current_vel = path.waypoints[i].twist.twist.linear.x;
    double vel_sign = (current_vel < 0) ? -1.0 : 1.0;

    // Clamp the current velocity within the calculated limits
    double vel_limited = current_vel;
    if (vel_sign > 0)
    {
      vel_limited = std::min(vel_limited, prev_vel + accel * travel_time);
      if (vel_limited < vel_min)
      {
        vel_limited = vel_min;
      }
    }
    else
    {
      vel_limited = std::max(vel_limited, prev_vel - accel * travel_time);
      if (vel_limited > -vel_min)
      {
        vel_limited = -vel_min;
      }
    }

    // Only update the velocity if it's above vel_min to prevent unnecessary changes
    if (std::fabs(current_vel) > vel_min)
    {
      path.waypoints[i].twist.twist.linear.x = vel_limited;
    }

    // Update previous velocity for the next iteration
    prev_vel = path.waypoints[i].twist.twist.linear.x;
  }

  // Limit deceleration
  double next_vel = path.waypoints.back().twist.twist.linear.x;
  for (int i = static_cast<int>(path.waypoints.size()) - 2; i >= 0; --i)
  {
    double dist =
        amathutils::find_distance(path.waypoints[i].pose.pose.position, path.waypoints[i + 1].pose.pose.position);

    double travel_time = 1.0;
    if (fabs(next_vel) > 0.0001)
    {
      travel_time = dist / fabs(next_vel);
    }

    // Calculate velocity limits based on deceleration
    double current_vel = path.waypoints[i].twist.twist.linear.x;
    double vel_sign = (current_vel < 0) ? -1.0 : 1.0;

    // Clamp the current velocity within the calculated limits
    double vel_limited = current_vel;
    if (vel_sign > 0)
    {
      vel_limited = std::min(vel_limited, next_vel + decel * travel_time);
      if (vel_limited < vel_min)
      {
        vel_limited = vel_min;
      }
    }
    else
    {
      vel_limited = std::max(vel_limited, next_vel - decel * travel_time);
      if (vel_limited > -vel_min)
      {
        vel_limited = -vel_min;
      }
    }

    // Only update the velocity if it's above vel_min to prevent unnecessary changes
    if (std::fabs(current_vel) > vel_min)
    {
      path.waypoints[i].twist.twist.linear.x = vel_limited;
    }

    // Update next velocity for the next iteration
    next_vel = path.waypoints[i].twist.twist.linear.x;
  }
}
