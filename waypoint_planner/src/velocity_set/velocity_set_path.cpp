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
#include <amathutils_lib/amathutils.hpp>

#include <algorithm>

VelocitySetPath::VelocitySetPath()
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<double>("min_accel_velocity", min_accel_velocity_, 1.0);
  private_nh_.param<double>("decelerate_vel_min", min_decel_velocity_, 1.3);
  private_nh_.param<float>("accel_velocity", accel_velocity_, 0.5);
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
  {
    return;
  }

  temporal_waypoints_.waypoints.clear();
  temporal_waypoints_.header = updated_waypoints_.header;
  temporal_waypoints_.increment = updated_waypoints_.increment;

  // push current pose
  autoware_msgs::Waypoint current_point;
  current_point.pose = control_pose;
  current_point.pose.pose = closest_path_pose_;
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
  // Use static variables to reduce multiplications when the function is called
  // repeatedly with the same current_vel arg
  static double current_velocity = current_vel;
  static double square_vel = current_vel * current_vel;
  if (current_velocity != current_vel)
  {
    current_velocity = current_vel;
    square_vel = current_vel * current_vel;
  }

  // accelerate with constant acceleration
  // v = root((v0)^2 + 2ax)
  return std::sqrt(square_vel + 2.0 * accel * distanceBetweenWaypoints(range.at(0), range.at(1)));
}

void VelocitySetPath::changeWaypointsForDeceleration(double deceleration, int closest_waypoint, int obstacle_waypoint)
{
  int extra = 4;  // for safety

  // decelerate with constant deceleration
  for (int index = obstacle_waypoint + extra; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index))
      continue;

    // v = sqrt( (v0)^2 + 2ax )
    // Keep the car at min_decel_velocity_ when approaching the obstacles.
    // without min_decel_velocity_ term, changed_vel becomes zero if index == obstacle_waypoint.
    std::array<int, 2> range = {index, obstacle_waypoint};
    double changed_vel = calcChangedVelocity(min_decel_velocity_, deceleration, range);

    double prev_vel = original_waypoints_.waypoints[index].twist.twist.linear.x;
    const int sgn = (prev_vel < 0) ? -1 : 1;
    updated_waypoints_.waypoints[index].twist.twist.linear.x = sgn * std::min(std::abs(prev_vel), changed_vel);
  }
}

void VelocitySetPath::updateClosestPathPose(geometry_msgs::Pose current_pose, int closest_wp_index)
{
  int point_a_index;
  int point_b_index;
  int wp_size = original_waypoints_.waypoints.size();
  closest_path_pose_.orientation = current_pose.orientation;

  if (wp_size == 1)
  {
    closest_path_pose_.position = current_pose.position;
    return;
  }

  // Find closest point on the path
  if (closest_wp_index <= 0)
  {
    point_a_index = 0;
    point_b_index = 1;
  }
  else if (closest_wp_index >= wp_size - 1)
  {
    point_a_index = wp_size - 2;
    point_b_index = wp_size - 1;
  }
  else
  {
    point_a_index = closest_wp_index;
    point_b_index = closest_wp_index + 1;
  }

  closest_path_pose_.position = amathutils::getNearPtOnLine(
    current_pose.position,
    original_waypoints_.waypoints[point_a_index].pose.pose.position,
    original_waypoints_.waypoints[point_b_index].pose.pose.position);
}

void VelocitySetPath::avoidSuddenAcceleration(double acceleration, int closest_wp_index)
{
  // Check if there is no need to accelerate
  const double original_vel = updated_waypoints_.waypoints[closest_wp_index].twist.twist.linear.x;
  if (current_vel_ >= original_vel)
  {
    return;
  }

  // Update closest/first point based on constant acceleration
  double accelerated_vel = current_vel_ + accel_velocity_;

  accelerated_vel = std::max(accelerated_vel, min_accel_velocity_);
  accelerated_vel = std::min(accelerated_vel, original_vel);

  const int sgn = (original_vel < 0) ? -1 : 1;
  updated_waypoints_.waypoints[closest_wp_index].twist.twist.linear.x = sgn * accelerated_vel;

  // Update the rest of the waypoints accelerating from closest point
  int starting_wp_index = closest_wp_index;
  for (int i = 1;; i++)
  {
    int wp_index = starting_wp_index + i;

    if (!checkWaypoint(wp_index))
    {
      return;
    }

    std::array<int, 2> range = {starting_wp_index, wp_index};
    double changed_vel = calcChangedVelocity(accelerated_vel, acceleration, range);
    changed_vel = std::max(changed_vel, min_accel_velocity_);
    const double original_vel = updated_waypoints_.waypoints[wp_index].twist.twist.linear.x;

    // Don't exceed original velocity
    if (changed_vel >= std::abs(original_vel))
    {
      return;
    }

    const int sgn = (original_vel < 0) ? -1 : 1;
    updated_waypoints_.waypoints[wp_index].twist.twist.linear.x = sgn * changed_vel;
  }
}

void VelocitySetPath::avoidSuddenDeceleration(double velocity_change_limit, double deceleration, int closest_waypoint)
{
  if (closest_waypoint < 0)
    return;

  const double closest_vel = updated_waypoints_.waypoints[closest_waypoint].twist.twist.linear.x;

  // if accelerating, do not modify the speed profile.
  if ((current_vel_ >= 0.0 && current_vel_ <= closest_vel) || (current_vel_ < 0.0 && current_vel_ > closest_vel))
    return;

  // if decelerating within the limit, do not modify the speed profile.
  if (std::abs(current_vel_ - closest_vel) < velocity_change_limit)
    return;

  // bring up the forward waypoints' velocity to avoid sudden deceleration.
  for (int i = 0;; i++)
  {
    if (!checkWaypoint(closest_waypoint + i))
      return;

    // sqrt(v^2 - 2ax)
    std::array<int, 2> range = {closest_waypoint, closest_waypoint + i};
    double changed_vel = calcChangedVelocity(std::abs(current_vel_) - velocity_change_limit, -deceleration, range);
    const double target_vel = updated_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x;

    if (std::isnan(changed_vel))
    {
      break;
    }
    const int sgn = (target_vel < 0) ? -1 : 1;
    updated_waypoints_.waypoints[closest_waypoint + i].twist.twist.linear.x = sgn * changed_vel;
  }
}

void VelocitySetPath::changeWaypointsForStopping(int stop_waypoint, int obstacle_waypoint,
                                                  int closest_waypoint, double deceleration)
{
  if (closest_waypoint < 0)
  {
    return;
  }

  // decelerate with constant deceleration
  for (int index = stop_waypoint; index >= closest_waypoint; index--)
  {
    if (!checkWaypoint(index))
    {
      continue;
    }

    // v = (v0)^2 + 2ax, and v0 = 0
    std::array<int, 2> range = {index, stop_waypoint};
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

double VelocitySetPath::distanceBetweenWaypoints(const int& begin, const int& end) const
{
  // Check index
  if (begin < 0 || begin >= getPrevWaypointsSize() || end < 0 || end >= getPrevWaypointsSize() || begin > end)
  {
    ROS_WARN_THROTTLE(1, "Invalid input index range");
    return 0.0;
  }

  // Calculate the distance between the waypoints
  double dist_sum = 0.0;
  for (int i = begin; i < end; i++)
  {
    dist_sum += distanceBetweenPoints(
      original_waypoints_.waypoints[i].pose.pose.position,
      original_waypoints_.waypoints[i + 1].pose.pose.position);
  }

  return dist_sum;
}

double VelocitySetPath::distanceBetweenPoints(const geometry_msgs::Point& begin, const geometry_msgs::Point& end) const
{
  // Calculate the distance between the waypoints
  tf::Vector3 v1(begin.x, begin.y, 0);
  tf::Vector3 v2(end.x, end.y, 0);

  return tf::tfDistance(v1, v2);
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
