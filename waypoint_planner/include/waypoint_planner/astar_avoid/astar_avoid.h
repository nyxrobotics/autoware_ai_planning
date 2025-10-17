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

#ifndef ASTAR_AVOID_H
#define ASTAR_AVOID_H

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/LaneArray.h>

#include "libwaypoint_follower/libwaypoint_follower.h"
#include "astar_search/astar_search.h"

class AstarAvoid
{
public:
  enum WayType : int8_t
  {
    RELAY = 0,
    AVOID = 1
  };

  enum AsterPlanStatus : int8_t
  {
    IDLE = 0,
    PLAN = 1,
    SUCCESS = 2,
    FAILURE = 3
  };

  AstarAvoid();
  ~AstarAvoid() = default;
  void run();

private:
  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher safety_waypoints_pub_;
  ros::Publisher debug_pub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber current_velocity_sub_;
  ros::Subscriber base_waypoints_sub_;
  ros::Subscriber closest_waypoint_sub_;
  ros::Subscriber obstacle_waypoint_sub_;
  ros::Subscriber state_sub_;
  ros::Rate* rate_;
  ros::Timer timer_;
  tf::TransformListener tf_listener_;

  // params
  int safety_waypoints_size_;  // output waypoint size [-]
  double update_rate_;         // publishing rate [Hz]

  bool enable_avoidance_;            // enable avoidance mode
  bool use_back_;                    // enable switchback action
  double avoid_waypoints_velocity_;  // constant velocity on planned waypoints [km/h]
  int plan_start_index_;             // start index for avoidance planning [-]
  double replan_interval_;           // replan interval for avoidance planning [Hz]
  int search_waypoints_size_;        // range of waypoints for incremental search [-]
  int search_waypoints_delta_;       // skipped waypoints for incremental search [-]
  int closest_search_size_;          // search closest waypoint around your car [-]
  int stopline_ahead_num_;
  double accel_limit_;  // acceleration limit [m/s^2]
  double decel_limit_;  // deceleration limit [m/s^2]
  double vel_min_;      // minimum velocity [km/h]

  // classes
  AstarSearch astar_;
  AsterPlanStatus astar_plan_status_;

  // variables
  bool found_avoid_path_;

  // Index of the closest waypoint in the current_waypoints_ Lane.
  // Not the same as the waypoint gid. This value can change suddenly if the
  // current_waypoints_ switches between base_waypoints_ and avoid_waypoints_.
  WayType select_way_;
  int base_index_ = -1;
  int avoid_index_ = -1;
  int avoid_start_base_index_ = -1;
  int avoid_path_size_ = -1;
  int avoid_finish_base_index_ = -1;

  // Index of the obstacle relative to current_waypoint_index_
  bool is_move_;
  bool found_obstacle_;
  int obstacle_index_ = -1;
  nav_msgs::OccupancyGrid costmap_;
  autoware_msgs::Lane base_waypoints_;
  autoware_msgs::Lane avoid_waypoints_;
  geometry_msgs::PoseStamped current_pose_local_, current_pose_global_;
  geometry_msgs::PoseStamped goal_pose_local_, goal_pose_global_;
  geometry_msgs::TwistStamped current_velocity_;
  tf::Transform local2costmap_;  // local frame (e.g. velodyne) -> costmap origin

  bool costmap_initialized_ = false;
  bool current_pose_initialized_ = false;
  bool current_velocity_initialized_ = false;
  bool base_waypoints_initialized_ = false;
  bool base_index_initialized_ = false;

  // functions, callback
  void costmapCallback(const nav_msgs::OccupancyGrid& msg);
  void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void currentVelocityCallback(const geometry_msgs::TwistStamped& msg);
  void baseWaypointsCallback(const autoware_msgs::Lane& msg);
  void closestIndexCallback(const std_msgs::Int32& msg);
  void obstacleIndexCallback(const std_msgs::Int32& msg);

  // functions
  bool checkInitialized();
  bool planAvoidWaypoints(int& end_of_avoid_index);
  void mergeAvoidWaypoints(const nav_msgs::Path& path, const int start_index, const int goal_index,
                           int& end_of_avoid_index);
  void mergeAvoidWaypoints(const nav_msgs::Path& path, const int start_index, const int goal_index,
                           int& end_of_avoid_index, tf::Transform base2avoid);
  tf::Transform getTransform(const std::string& from, const std::string& to);
  // publish safety waypoints using a timer
  void publishWaypoints(const ros::TimerEvent& e);
  void limitPathAccel(autoware_msgs::Lane& path, double accel, double decel, double vel_min);
};

#endif
