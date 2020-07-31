// Copyright 2018-2020 Autoware Foundation. All rights reserved.
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

#include "decision_maker/decision_maker_node.h"

#include <cstdio>
#include <numeric>
#include <vector>
#include <utility>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

#include <autoware_msgs/Lane.h>
#include <autoware_msgs/TrafficLight.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <std_msgs/String.h>

namespace decision_maker
{

double DecisionMakerNode::calcIntersectWayAngle(const autoware_msgs::Lane& laneinArea)
{
  double diff = 0.0;
  if (laneinArea.waypoints.empty())
  {
    ROS_INFO("Not inside CrossRoad");
  }
  else
  {
    const geometry_msgs::Pose InPose = laneinArea.waypoints.front().pose.pose;
    const geometry_msgs::Pose OutPose = laneinArea.waypoints.back().pose.pose;

    diff = amathutils::calcPosesAngleDiffDeg(InPose, OutPose);
  }

  return diff;
}

double DecisionMakerNode::getDistToWaypointIdx(const int wpidx) const
{
  double distance = 0.0;
  geometry_msgs::Pose prev_pose = current_status_.pose;

  for (unsigned int idx = 0; idx < current_status_.finalwaypoints.waypoints.size() - 1; idx++)
  {
    distance += amathutils::find_distance(prev_pose, current_status_.finalwaypoints.waypoints.at(idx).pose.pose);

    if (current_status_.finalwaypoints.waypoints.at(idx).gid == wpidx)
    {
      break;
    }

    prev_pose = current_status_.finalwaypoints.waypoints.at(idx).pose.pose;
  }

  return distance;
}

double DecisionMakerNode::calcRequiredDistForStop(void) const
{
  static const double mu = 0.7;  // dry ground/ asphalt/ normal tire
  static const double g = 9.80665;
  static const double margin = 5;
  static const double reaction_time = 0.3 + margin;  // system delay(sec)
  const double velocity = current_status_.velocity;

  const double free_running_distance = reaction_time * velocity;
  const double braking_distance = velocity * velocity / (2 * g * mu);
  const double distance_to_target = (free_running_distance + braking_distance) * 2 /* safety margin*/;

  return distance_to_target;
}

// Assign unique gid and lid for each waypoint in the received based_lane_array, and
// append additional waypoint state information based on the map format. At the same,
// the function return the distance and the heading difference in between the
// ego-vehicle and the closest waypoint in a std::pair.
std::pair<double, double> DecisionMakerNode::prepareActiveLaneArray()
{
  int gid = 0;
  for (auto& lane : current_status_.based_lane_array.lanes)
  {
    int lid = 0;
    for (auto& wp : lane.waypoints)
    {
      wp.wpstate.aid = 0;
      wp.wpstate.steering_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.accel_state = autoware_msgs::WaypointState::NULLSTATE;
      if (wp.wpstate.stop_state != autoware_msgs::WaypointState::TYPE_STOPLINE &&
          wp.wpstate.stop_state != autoware_msgs::WaypointState::TYPE_STOP)
      {
        wp.wpstate.stop_state = autoware_msgs::WaypointState::NULLSTATE;
      }

      wp.wpstate.lanechange_state = autoware_msgs::WaypointState::NULLSTATE;
      wp.wpstate.event_state = 0;
      wp.gid = gid++;
      wp.lid = lid++;

      if (!isEventFlagTrue("received_back_state_waypoint") && wp.twist.twist.linear.x < 0.0)
      {
        setEventFlag("received_back_state_waypoint", true);
        publishOperatorHelpMessage("Received back waypoint.");
      }
    }
  }

  // set waypoint state and insert interpolated waypoint for stopline if necessary
  if (!ignore_map_)
  {
    if (use_lanelet_map_)
    {
      setWaypointStateUsingLanelet2Map(current_status_.based_lane_array);
    }
    else
    {
      setWaypointStateUsingVectorMap(current_status_.based_lane_array);
    }
  }

  // re-index since setWaypointStateUsingLanelet2Map() or setWaypointStateUsingVectorMap()
  // may change the size of waypoints.
  gid = 0;
  double min_dist = DBL_MAX;
  double heading_diff = DBL_MAX;
  geometry_msgs::Pose closest_wp_pose;
  for (auto& lane : current_status_.based_lane_array.lanes)
  {
    int lid = 0;
    for (auto& wp : lane.waypoints)
    {
      wp.gid = gid++;
      wp.lid = lid++;
      const double dist = amathutils::find_distance(current_status_.pose.position, wp.pose.pose.position);
      if (min_dist > dist)
      {
        min_dist = dist;
        closest_wp_pose = wp.pose.pose;
      }
    }
  }

  if (min_dist != DBL_MAX)
  {
    heading_diff = std::abs(amathutils::calcPosesAngleDiffDeg(current_status_.pose, closest_wp_pose));
  }

  return std::pair<double, double>(min_dist, heading_diff);
}

bool DecisionMakerNode::isLocalizationConvergence(const geometry_msgs::Point& _current_point) const
{
  static std::vector<double> distances;
  static uint32_t distances_count = 0;
  static geometry_msgs::Point prev_point;
  static const int param_convergence_count = 10;

  bool ret = false;

  // if current point is not set, localization is failure
  if (_current_point.x == 0 && _current_point.y == 0 && _current_point.z == 0 && prev_point.x == 0 &&
      prev_point.y == 0 && prev_point.z == 0)
  {
    return ret;
  }

  distances.push_back(amathutils::find_distance(prev_point, _current_point));
  if (++distances_count > param_convergence_count) /* num of count to judge convergence*/
  {
    distances.erase(distances.begin());
    distances_count--;
    double avg_distances =
      std::accumulate(distances.begin(), distances.end(), 0.0) / static_cast<double>(distances.size());
    if (avg_distances <= 2) /*meter*/
    {
      ret = true;
    }
  }

  prev_point = _current_point;
  return ret;
}

bool DecisionMakerNode::isArrivedGoal() const
{
  const auto goal_point = current_status_.finalwaypoints.waypoints.back().pose.pose.position;

  if (amathutils::find_distance(goal_point, current_status_.pose.position) < goal_threshold_dist_
      && fabs(current_status_.velocity) <= goal_threshold_vel_)
  {
    return true;
  }

  return false;
}

}  // namespace decision_maker
