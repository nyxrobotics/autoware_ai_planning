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

#include <waypoint_planner/velocity_set/velocity_set_path.h>

VelocitySetPath::VelocitySetPath()
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<double>("velocity_offset", velocity_offset_, 1.2);
  private_nh_.param<double>("decelerate_vel_min", decelerate_vel_min_, 1.3);
}

// check if waypoint number is valid
bool VelocitySetPath::checkWaypoint(int wp_num) const
{
  if (wp_num < 0 || wp_num >= getPrevWaypointsSize())
  {
    return false;
  }
  return true;
}

// set about '_temporal_waypoints_size' meter waypoints from closest waypoint
void VelocitySetPath::setTemporalWaypoints(int temporal_waypoints_size, int closest_waypoint,
                                           geometry_msgs::PoseStamped control_pose)
{
  if (closest_waypoint < 0)
    return;

  temporal_waypoints_.waypoints.clear();
  temporal_waypoints_.header = updated_waypoints_.header;
  temporal_waypoints_.increment = updated_waypoints_.increment;

  // push current pose
  autoware_msgs::Waypoint current_point;
  current_point.pose = control_pose;
  current_point.twist = updated_waypoints_.waypoints[closest_waypoint].twist;
  current_point.dtlane = updated_waypoints_.waypoints[closest_waypoint].dtlane;
  temporal_waypoints_.waypoints.push_back(std::move(current_point));

  int total_waypoints = getNewWaypointsSize();
  for (int i = 0; i < temporal_waypoints_size; i++)
  {
    if (closest_waypoint + i >= total_waypoints)
      return;

    temporal_waypoints_.waypoints.push_back(updated_waypoints_.waypoints[closest_waypoint + i]);
  }

  return;
}

double VelocitySetPath::calcChangedVelocity(const double& current_vel, const double& accel,
                                            const std::array<int, 2>& range) const
{
  static double current_velocity = current_vel;
  static double square_vel = current_vel * current_vel;
  if (current_velocity != current_vel)
  {
    current_velocity = current_vel;
    square_vel = current_vel * current_vel;
  }
  return std::sqrt(square_vel + 2.0 * accel * calcInterval(range.at(0), range.at(1)));
}

void VelocitySetPath::changeWaypointsForDeceleration(double deceleration, int closest_waypoint, int obstacle_waypoint)
{
  int extra = 4;  // for safety

  // decelerate with constant deceleration
  for (int index = obstacle_waypoint + extra; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index))
      continue;
    if (index > obstacle_waypoint)
    {
      // After obstacle_waypoint, set the speed of extra points to decelerate_vel_min_.
      updated_waypoints_.waypoints[index].twist.twist.linear.x = decelerate_vel_min_;
      continue;
    }
    // v = sqrt( (v0)^2 + 2ax )
    // Keep the car at decelerate_vel_min_ when approaching the obstacles.
    // without decelerate_vel_min_ term, changed_vel becomes zero if index == obstacle_waypoint.
    std::array<int, 2> range = { index, obstacle_waypoint };
    double changed_vel = calcChangedVelocity(decelerate_vel_min_, deceleration, range);

    double prev_vel = original_waypoints_.waypoints[index].twist.twist.linear.x;
    const int sgn = (prev_vel < 0) ? -1 : 1;
    updated_waypoints_.waypoints[index].twist.twist.linear.x = sgn * std::min(std::abs(prev_vel), changed_vel);
  }
}

void VelocitySetPath::avoidSuddenAcceleration(double deceleration, int closest_waypoint)
{
  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i))
      return;

    // accelerate with constant acceleration
    // v = root((v0)^2 + 2ax)
    // Without velocity_offset_ term, changed_vel becomes current_vel_ when i == 0. For example, the car will not move
    // if current_vel_ == 0.
    std::array<int, 2> range = { closest_waypoint, closest_waypoint + i };
    double changed_vel = calcChangedVelocity(current_vel_, deceleration, range) + velocity_offset_;

    const double target_vel = updated_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x;
    // Don't exceed original velocity
    if (changed_vel > std::abs(target_vel))
      return;

    const int sgn = (target_vel < 0) ? -1 : 1;
    updated_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = sgn * changed_vel;
  }

  return;
}

void VelocitySetPath::avoidSuddenDeceleration(double velocity_change_limit, double deceleration, int closest_waypoint)
{
  constexpr double epsilon = 1e-9;

  if (closest_waypoint < 0)
    return;
  if (!checkWaypoint(closest_waypoint))
    return;

  const double closest_vel = updated_waypoints_.waypoints[closest_waypoint].twist.twist.linear.x;

  // --------------------------------------------------------------------------
  // Part 1) Limit deceleration between current_vel_ and closest_vel (accel is not limited)
  // --------------------------------------------------------------------------
  double closest_length = 0.0;
  if (closest_waypoint + 1 < getNewWaypointsSize())
    closest_length = calcInterval(closest_waypoint, closest_waypoint + 1);

  if (closest_length > epsilon)
  {
    // Estimate accel needed to reach closest_vel from current_vel_ over closest_length.
    // Use v^2 = v0^2 + 2 a s (same-sign), or crossing-zero form when signs differ.
    double closest_accel = 0.0;
    if (closest_vel * current_vel_ > 0.0)
    {
      closest_accel = (closest_vel * closest_vel - current_vel_ * current_vel_) / (2.0 * closest_length);
    }
    else
    {
      closest_accel = (closest_vel * closest_vel + current_vel_ * current_vel_) / (2.0 * closest_length);
      if (closest_vel < current_vel_)
        closest_accel = -std::abs(closest_accel);
    }

    // Only limit *deceleration*; acceleration is not limited.
    const bool decel_step = (std::abs(closest_vel) < std::abs(current_vel_)) || (current_vel_ * closest_vel < 0.0);

    if (decel_step && std::abs(closest_accel) > std::abs(velocity_change_limit))
    {
      // Clamp accel magnitude to the allowed change limit (keep sign)
      closest_accel = std::copysign(std::abs(velocity_change_limit), closest_accel);

      // Recompute velocity at the closest waypoint from the clamped accel
      auto& v_out = updated_waypoints_.waypoints[closest_waypoint].twist.twist.linear.x;

      const bool same_sign = (closest_vel >= 0.0 && current_vel_ >= 0.0) || (closest_vel <= 0.0 && current_vel_ <= 0.0);

      double sq_term = same_sign ? (current_vel_ * current_vel_ + 2.0 * closest_accel * closest_length) :
                                   (2.0 * closest_accel * closest_length - current_vel_ * current_vel_);

      sq_term = std::max(0.0, sq_term);  // numerical guard
      const double speed_mag = std::sqrt(sq_term);
      v_out = (closest_vel >= 0.0) ? speed_mag : -speed_mag;
    }
  }

  // --------------------------------------------------------------------------
  // Part 2) Forward profile construction
  //   - No accel limiting: only raise sudden dips (deceleration) when there is no mandatory stop/flip.
  //   - With a mandatory stop/flip ahead: cap by the *minimum* decel envelope that guarantees stopping.
  //   - Iterate from the stop/flip point backwards to the current point as requested.
  // --------------------------------------------------------------------------

  // Find first must-stop waypoint (near zero) ahead
  int stop_index = -1;
  for (int i = closest_waypoint; i < getNewWaypointsSize(); ++i)
  {
    if (!checkWaypoint(i))
      return;
    const double v = updated_waypoints_.waypoints[i].twist.twist.linear.x;
    if (std::abs(v) < 1e-3)
    {
      stop_index = i;
      break;
    }
  }

  // Find first direction-change waypoint ahead (sign flip versus current motion)
  int dir_change_index = -1;
  const int sign_now = (current_vel_ > 0.0) ? 1 : (current_vel_ < 0.0 ? -1 : 0);
  if (sign_now != 0)
  {
    for (int i = closest_waypoint; i < getNewWaypointsSize(); ++i)
    {
      if (!checkWaypoint(i))
        return;
      const double v = updated_waypoints_.waypoints[i].twist.twist.linear.x;
      const int sv = (v > 0.0) ? 1 : (v < 0.0 ? -1 : 0);
      if (sv != 0 && sv != sign_now)
      {
        dir_change_index = i;
        break;
      }
    }
  }

  // Choose nearest mandatory stop/flip target (closest from current)
  int stop_target_index = -1;
  double stop_dist_from_closest = std::numeric_limits<double>::infinity();
  if (stop_index != -1)
  {
    stop_target_index = stop_index;
    stop_dist_from_closest = calcInterval(closest_waypoint, stop_index);
  }
  if (dir_change_index != -1)
  {
    const double d_flip = calcInterval(closest_waypoint, dir_change_index);
    if (d_flip < stop_dist_from_closest)
    {
      stop_target_index = dir_change_index;
      stop_dist_from_closest = d_flip;
    }
  }

  // Build decel envelope
  const double v0_mag_for_stop = std::abs(updated_waypoints_.waypoints[closest_waypoint].twist.twist.linear.x);
  const bool have_mandatory_stop = (stop_target_index != -1) && std::isfinite(stop_dist_from_closest) &&
                                   (stop_dist_from_closest > epsilon) && (v0_mag_for_stop > 0.0);

  // Effective decel magnitude:
  //  - normal: |velocity_change_limit|
  //  - mandatory stop/flip: ensure enough decel to reach zero within stop_dist_from_closest
  double a_eff_mag = std::abs(velocity_change_limit);
  if (have_mandatory_stop)
  {
    const double required_decel = (v0_mag_for_stop * v0_mag_for_stop) / (2.0 * stop_dist_from_closest);
    a_eff_mag = std::max(a_eff_mag, required_decel);  // may exceed limit to guarantee stop
  }

  const double v0_mag = v0_mag_for_stop;

  if (have_mandatory_stop)
  {
    // ---- Backward loop: from stop/flip -> closest (as requested) ----
    double d_acc = 0.0;  // distance from current idx to stop_target (accumulated while going backward)

    for (int idx = stop_target_index; idx >= closest_waypoint; --idx)
    {
      if (!checkWaypoint(idx))
        return;

      const double d_rem = d_acc;
      const double s_acc = std::max(0.0, stop_dist_from_closest - d_rem);

      // forward envelope from closest: v_fwd(s) = sqrt(max(0, v0^2 - 2 a s))
      double v_env_mag_fwd = std::sqrt(std::max(0.0, v0_mag * v0_mag - 2.0 * a_eff_mag * s_acc));
      // backward envelope from stop:  v_stop(d) = sqrt(max(0, 2 a d))
      double v_env_mag_bwd = std::sqrt(std::max(0.0, 2.0 * a_eff_mag * d_rem));

      // To guarantee stopping, cap by the tighter of the two
      double v_env_mag = std::min(v_env_mag_fwd, v_env_mag_bwd);

      // Force zero exactly at the stop/flip index
      if (idx == stop_target_index)
        v_env_mag = 0.0;

      // planned target at idx
      const double target_vel = updated_waypoints_.waypoints[idx].twist.twist.linear.x;
      const double target_mag = std::abs(target_vel);
      const int sgn = (target_vel < 0.0) ? -1 : 1;

      // With a mandatory stop, profile must be <= envelope
      const double new_mag = std::min(target_mag, v_env_mag);
      const double new_vel = sgn * new_mag;
      updated_waypoints_.waypoints[idx].twist.twist.linear.x = new_vel;

      // Early exit:
      //  - envelope zero and new velocity zero (stop), OR
      //  - sign flips relative to current motion (treat as stop boundary)
      const int new_sign = (new_vel > 0.0) ? 1 : (new_vel < 0.0 ? -1 : 0);
      if ((v_env_mag <= 0.0 && new_mag <= 0.0) || (sign_now != 0 && new_sign != 0 && new_sign != sign_now))
        break;

      // accumulate remaining distance for the next (previous) index
      if (idx > closest_waypoint)
      {
        d_acc += calcInterval(idx - 1, idx);  // distance from (idx-1) to current idx
      }
    }
  }
  else
  {
    // ---- No mandatory stop/flip: backward pass that enforces pairwise deceleration limit only ----
    // We do NOT limit acceleration. We only slow down earlier points if (i-1)->i would exceed decel limit.
    const int last_idx = getNewWaypointsSize() - 1;

    // If motion has a known sign, stop the pass at the first sign change ahead.
    int end_idx = last_idx;
    if (sign_now != 0)
    {
      for (int i = closest_waypoint + 1; i <= last_idx; ++i)
      {
        if (!checkWaypoint(i))
          return;
        const double v = updated_waypoints_.waypoints[i].twist.twist.linear.x;
        const int sv = (v > 0.0) ? 1 : (v < 0.0 ? -1 : 0);
        if (sv != 0 && sv != sign_now)
        {
          end_idx = i - 1;
          break;
        }
      }
    }

    const double a_lim = std::abs(velocity_change_limit);

    for (int i = end_idx; i > closest_waypoint; --i)
    {
      if (!checkWaypoint(i) || !checkWaypoint(i - 1))
        return;

      const double ds = calcInterval(i - 1, i);
      if (ds <= epsilon)
        continue;

      const double v_next = updated_waypoints_.waypoints[i].twist.twist.linear.x;
      const double v_curr = updated_waypoints_.waypoints[i - 1].twist.twist.linear.x;

      // Respect sign if known; skip segments that already flip sign (handled elsewhere)
      if (sign_now != 0)
      {
        const int s_next = (v_next > 0.0) ? 1 : (v_next < 0.0 ? -1 : 0);
        const int s_curr = (v_curr > 0.0) ? 1 : (v_curr < 0.0 ? -1 : 0);
        if ((s_next != 0 && s_next != sign_now) || (s_curr != 0 && s_curr != sign_now))
          break;
      }

      // Allowed speed magnitude at (i-1) to not exceed decel limit into i:
      // v_{i-1} <= sqrt( v_i^2 + 2 * a_lim * ds )
      const double v_allowed_mag = std::sqrt(std::max(0.0, v_next * v_next + 2.0 * a_lim * ds));

      // If current magnitude is higher than allowed, lower it (do NOT raise = no accel limit)
      const double v_curr_mag = std::abs(v_curr);
      if (v_curr_mag > v_allowed_mag)
      {
        const int sgn = (v_curr < 0.0) ? -1 : 1;
        updated_waypoints_.waypoints[i - 1].twist.twist.linear.x = sgn * v_allowed_mag;
      }
    }
  }
}

void VelocitySetPath::changeWaypointsForStopping(int stop_waypoint, int obstacle_waypoint, int closest_waypoint,
                                                 double deceleration)
{
  if (closest_waypoint < 0)
    return;

  // decelerate with constant deceleration
  for (int index = stop_waypoint; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index))
      continue;

    // v = (v0)^2 + 2ax, and v0 = 0
    std::array<int, 2> range = { index, stop_waypoint };
    const double changed_vel = calcChangedVelocity(0.0, deceleration, range);
    const double prev_vel = original_waypoints_.waypoints[index].twist.twist.linear.x;
    const int sgn = (prev_vel < 0) ? -1 : 1;
    updated_waypoints_.waypoints[index].twist.twist.linear.x = sgn * std::min(std::abs(prev_vel), changed_vel);
  }

  // fill velocity with 0 for stopping waypoint and the rest.
  for (auto it = updated_waypoints_.waypoints.begin() + stop_waypoint; it != updated_waypoints_.waypoints.end(); ++it)
  {
    it->twist.twist.linear.x = 0.0;
  }
}

void VelocitySetPath::initializeNewWaypoints()
{
  updated_waypoints_ = original_waypoints_;
}

double VelocitySetPath::calcInterval(const int begin, const int end) const
{
  // check index
  if (begin < 0 || begin >= getPrevWaypointsSize() || end < 0 || end >= getPrevWaypointsSize() || begin > end)
  {
    ROS_WARN("Invalid input index range: begin = %d, end = %d, PrevWaypointsSize = %d", begin, end,
             getPrevWaypointsSize());
    return 0.0;
  }

  // Calculate the inteval of waypoints
  double dist_sum = 0.0;
  for (int i = begin; i < end; i++)
  {
    tf::Vector3 v1(original_waypoints_.waypoints[i].pose.pose.position.x,
                   original_waypoints_.waypoints[i].pose.pose.position.y, 0);

    tf::Vector3 v2(original_waypoints_.waypoints[i + 1].pose.pose.position.x,
                   original_waypoints_.waypoints[i + 1].pose.pose.position.y, 0);

    dist_sum += tf::tfDistance(v1, v2);
  }

  return dist_sum;
}

void VelocitySetPath::resetFlag()
{
  set_path_ = false;
}

void VelocitySetPath::waypointsCallback(const autoware_msgs::LaneConstPtr& msg)
{
  original_waypoints_ = *msg;
  // temporary, edit waypoints velocity later
  updated_waypoints_ = *msg;

  set_path_ = true;
}

void VelocitySetPath::currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr& msg)
{
  current_vel_ = msg->twist.linear.x;
}
