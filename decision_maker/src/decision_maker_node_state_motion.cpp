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

#include <utility>
#include <algorithm>

namespace decision_maker
{
void DecisionMakerNode::updateWaitDriveReadyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateWaitEngageState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateMotionEmergencyState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::entryDriveState(cstring_t& state_name, int status)
{
  std::pair<uint8_t, int> get_stopsign = getStopSignStateFromWaypoint();
  if (get_stopsign.first != 0)
  {
    current_status_.found_stopsign_idx = get_stopsign.second;
  }

  if (current_status_.found_stopsign_idx != -1 || current_status_.ordered_stop_idx != -1)
  {
    tryNextState("found_stop_decision");
  }
  else
  {
    tryNextState("clear");
  }
}

void DecisionMakerNode::updateDriveState(cstring_t& state_name, int status)
{
  if (isArrivedGoal())
  {
    publishOperatorHelpMessage("Arrived at goal point");
    tryNextState("arrived_goal");
    return;
  }

  if (current_status_.closest_waypoint == -1 && current_status_.autonomy_engaged)
  {
    publishOperatorHelpMessage("The vehicle passed last waypoint or waypoint does not exist near the vehicle.");
    tryNextState("mission_aborted");
    return;
  }

  if (current_status_.finalwaypoints.waypoints.empty())
  {
    ROS_WARN("\"/final_waypoints.\" does not contain waypoints");
    return;
  }
}

// Iterate every waypoint in received /final_waypoints and find the first
// stop sign waypoint within certain search distance. If it is found, its
// gid and stop sign state (either TYPE_STOPLINE or TYPE_STOP) is returned
// as a std::pair. Otherwise, a pair of (NULLSTATE, -1) is returned.
std::pair<uint8_t, int> DecisionMakerNode::getStopSignStateFromWaypoint(void)
{
  static const double mu = 0.7;  // dry ground/ asphalt/ normal tire
  static const double g = 9.80665;
  static const double margin = 5;
  static const double reaction_time = 0.3 + margin;  // system delay(sec)
  const double velocity = current_status_.velocity;

  const double free_running_distance = reaction_time * velocity;
  const double braking_distance = velocity * velocity / (2 * g * mu);
  const double distance_to_target = (free_running_distance + braking_distance) * 2 /* safety margin*/;

  std::pair<uint8_t, int> ret(autoware_msgs::WaypointState::NULLSTATE, -1);

  double distance = 0.0;
  geometry_msgs::Pose prev_pose = current_status_.pose;

  // Index 0 holds ego-vehicle's current pose
  if (current_status_.finalwaypoints.waypoints.size() < 3)
  {
    return ret;
  }

  // start from index 1 since index 0 holds ego-vehicle's current pose.
  for (size_t idx = 1; idx < current_status_.finalwaypoints.waypoints.size() - 1; ++idx)
  {
    const autoware_msgs::Waypoint &current_wpt = current_status_.finalwaypoints.waypoints.at(idx);
    distance += amathutils::find_distance(prev_pose, current_wpt.pose.pose);

    if (current_wpt.wpstate.stop_state != autoware_msgs::WaypointState::NULLSTATE)
    {
      if (current_status_.curr_stopped_idx != current_wpt.gid)
      {
        ret.first = current_wpt.wpstate.stop_state;
        ret.second = current_wpt.gid;
        break;
      }
    }

    if (distance > distance_to_target)
    {
      break;
    }

    prev_pose = current_wpt.pose.pose;
  }
  return ret;
}

void DecisionMakerNode::entryGoState(cstring_t& state_name, int status)
{
  publishStoplineWaypointIdx(-1);
}
void DecisionMakerNode::updateGoState(cstring_t& state_name, int status)
{
  std::pair<uint8_t, int> get_stopsign = getStopSignStateFromWaypoint();
  if (get_stopsign.first != 0 && get_stopsign.first != current_status_.curr_stopped_idx)
  {
    current_status_.found_stopsign_idx = get_stopsign.second;
  }

  int obstacle_waypoint_gid = current_status_.obstacle_waypoint + current_status_.closest_waypoint;

  if (get_stopsign.first != 0 && current_status_.found_stopsign_idx != -1)
  {
    if (current_status_.obstacle_waypoint == -1 || current_status_.found_stopsign_idx <= obstacle_waypoint_gid)
    {
      tryNextState("found_stop_decision");
    }
  }

  if (current_status_.ordered_stop_idx != -1 &&
      calcRequiredDistForStop() > getDistToWaypointIdx(current_status_.ordered_stop_idx))
  {
    if (current_status_.obstacle_waypoint == -1 || current_status_.ordered_stop_idx <= obstacle_waypoint_gid)
    {
      tryNextState("found_stop_decision");
    }
  }
}

void DecisionMakerNode::updateWaitState(cstring_t& state_name, int status)
{
  publishStoplineWaypointIdx(current_status_.finalwaypoints.waypoints.at(2).gid);
}

void DecisionMakerNode::updateStopState(cstring_t& state_name, int status)
{
  int obstacle_waypoint_gid = current_status_.obstacle_waypoint + current_status_.closest_waypoint;
  std::pair<uint8_t, int> get_stopsign = getStopSignStateFromWaypoint();
  if (get_stopsign.first != 0)
  {
    current_status_.found_stopsign_idx = get_stopsign.second;
  }

  if (current_status_.obstacle_waypoint != -1)
  {
    if ((current_status_.found_stopsign_idx != -1 && current_status_.found_stopsign_idx >= obstacle_waypoint_gid)
        || (current_status_.ordered_stop_idx != -1 && current_status_.ordered_stop_idx >= obstacle_waypoint_gid))
      tryNextState("clear");
  }

  if (get_stopsign.first != 0 && current_status_.found_stopsign_idx != -1)
  {
    if (current_status_.ordered_stop_idx == -1 ||
        current_status_.found_stopsign_idx < current_status_.ordered_stop_idx)
    {
      switch (get_stopsign.first)
      {
        // conditional release of stop
        case autoware_msgs::WaypointState::TYPE_STOPLINE:
          tryNextState("found_stopline");
          break;
        // manual release of stop
        case autoware_msgs::WaypointState::TYPE_STOP:
          tryNextState("found_reserved_stop");
          break;
        default:
          break;
      }

      return;
    }
  }

  if (current_status_.ordered_stop_idx != -1)
  {
    if (current_status_.found_stopsign_idx == -1 ||
        current_status_.ordered_stop_idx <= current_status_.found_stopsign_idx)
    {
      tryNextState("received_stop_order");
      return;
    }
  }
}

void DecisionMakerNode::updateStoplineState(cstring_t& state_name, int status)
{
  publishStoplineWaypointIdx(current_status_.found_stopsign_idx);

  // only run this once when approaching an intersection
  if (!stopline_init_phase1_flag_ && current_status_.found_stopsign_idx != -1)
  {
    int stop_line_id = -1;
    current_status_.stopline_intersect_id = -1;
    current_status_.current_intersection_ptr = nullptr;
    // grab upcoming waypoint's stopline id
    for (const auto& wp : current_status_.finalwaypoints.waypoints)
    {
      if (wp.gid == current_status_.found_stopsign_idx)
      {
        stop_line_id = wp.stop_line_id;
        break;
      }
    }
    // find approaching intersect
    for (auto& intersect : intersects_)
    {
      for (auto& stop_area : intersect.stop_areas)
      {
        if (stop_area.stopline_id == stop_line_id)
        {
          current_status_.current_intersection_ptr = &intersect;
          // store intersection id that contains the approaching stopline
          current_status_.stopline_intersect_id = intersect.id;
          stop_area.is_safe = -1;  // ignore stop_area associated with approaching stopline
        }
        else
        {
          stop_area.is_safe = 0;  // not safe by default
        }
      }
      if (current_status_.current_intersection_ptr != nullptr)
      {
        break;
      }
    }
    current_status_.curr_stopped_idx = current_status_.found_stopsign_idx;
    stopline_init_phase1_flag_ = true;
  }

  if (fabs(current_status_.velocity) <= stopped_vel_
      // vehicle speed is lower than threshold set to determine stopped velocity
      && current_status_.stopline_waypoint != -1
      // if the stopline is still the route
      && (current_status_.stopline_waypoint + current_status_.closest_waypoint) == current_status_.found_stopsign_idx)
      // wp of approaching stopline + wp traveled = wp's gid/lid of stopline
  {
    // start timer once
    if (current_status_.stopline_wait_timer.isZero() && current_status_.stopline_safety_timer.isZero())
    {
      current_status_.stopline_wait_timer = ros::Time::now();
      current_status_.stopline_safety_timer = ros::Time::now();
      ROS_INFO("Starting stopline timers. Wait for %f seconds \n", stopline_wait_duration_);
    }

    ros::Duration tdiff_wait = ros::Time::now() - current_status_.stopline_wait_timer;
    ros::Duration tdiff_safety = ros::Time::now() - current_status_.stopline_safety_timer;
    // if timer hasn't reached seconds defined by stopline_wait_duration_
    if (tdiff_wait.toSec() < stopline_wait_duration_)
    {
      ROS_DEBUG("Wait for %f seconds... Calculated time diff: %f \n", stopline_wait_duration_, tdiff_wait.toSec());
    }
    // waiting for clearance of objects at stop areas
    if (tdiff_safety.toSec() < stopline_min_safety_duration_)
    {
      ROS_DEBUG("Objects detected in stop areas. Not safe to enter! [ safety_timer: %f / %f ]",
        tdiff_safety.toSec(), stopline_min_safety_duration_);
    }

    // if timer reaches time in seconds defined by stopline_wait_duration_
    // and passes safety_timer defined by stopline_min_safety_duration_
    if (tdiff_wait.toSec() >= stopline_wait_duration_ && tdiff_safety.toSec() >= stopline_min_safety_duration_)
    {
      if (current_status_.ordered_stop_idx != -1)
      {
        tryNextState("received_stop_order");
      }
      else
      {
        ROS_INFO("Intersection clear! Proceed");
        tryNextState("clear");
      }
      // reset timers
      current_status_.stopline_wait_timer = ros::Time(0);
      current_status_.stopline_safety_timer = ros::Time(0);
      // reset all flags
      stopline_init_phase1_flag_ = false;
      stopline_init_phase2_flag_ = false;
    }
  }
}

void DecisionMakerNode::updateOrderedStopState(cstring_t& state_name, int status)
{
  if (current_status_.ordered_stop_idx == -1 || current_status_.closest_waypoint > current_status_.ordered_stop_idx)
  {
    tryNextState("clear");
  }
  else
  {
    publishStoplineWaypointIdx(current_status_.ordered_stop_idx);
  }
}

void DecisionMakerNode::exitOrderedStopState(cstring_t& state_name, int status)
{
  if (current_status_.found_stopsign_idx == -1 || current_status_.ordered_stop_idx < current_status_.found_stopsign_idx)
  {
    current_status_.ordered_stop_idx = -1;
  }
}

void DecisionMakerNode::updateReservedStopState(cstring_t& state_name, int status)
{
  publishStoplineWaypointIdx(current_status_.found_stopsign_idx);
}
void DecisionMakerNode::exitReservedStopState(cstring_t& state_name, int status)
{
  current_status_.found_stopsign_idx = -1;
}

}  // namespace decision_maker
