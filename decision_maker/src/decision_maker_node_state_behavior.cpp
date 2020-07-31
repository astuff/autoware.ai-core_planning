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

namespace decision_maker
{
void DecisionMakerNode::updateStoppingState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateBehaviorEmergencyState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_HAZARD);
}
void DecisionMakerNode::exitBehaviorEmergencyState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_CLEAR);
}

void DecisionMakerNode::updateMovingState(cstring_t& state_name, int status)
{
  if (isVehicleOnLaneArea())
  {
    tryNextState("on_lane_area");
  }
  else
  {
    tryNextState("on_free_area");
  }
}

uint8_t DecisionMakerNode::getSteeringStateFromWaypoint(void)
{
  double distance = 0.0;
  geometry_msgs::Pose prev_pose = current_status_.pose;
  uint8_t state = autoware_msgs::WaypointState::NULLSTATE;

  // final waypoints index 0 holds ego-vehicle's current pose.
  for (unsigned int idx = 1; idx < current_status_.finalwaypoints.waypoints.size() - 1; idx++)
  {
    distance += amathutils::find_distance(prev_pose, current_status_.finalwaypoints.waypoints.at(idx).pose.pose);

    state = current_status_.finalwaypoints.waypoints.at(idx).wpstate.steering_state;

    if (state != autoware_msgs::WaypointState::NULLSTATE &&
        (state != autoware_msgs::WaypointState::STR_STRAIGHT || distance >= lookahead_distance_))
    {
      break;
    }
    prev_pose = current_status_.finalwaypoints.waypoints.at(idx).pose.pose;
  }
  return state;
}

uint8_t DecisionMakerNode::getEventStateFromWaypoint(void)
{
  double distance = 0.0;
  geometry_msgs::Pose prev_pose = current_status_.pose;
  uint8_t state = autoware_msgs::WaypointState::NULLSTATE;

  // final waypoints index 0 holds ego-vehicle's current pose.
  for (unsigned int idx = 1; idx < current_status_.finalwaypoints.waypoints.size() - 1; idx++)
  {
    distance += amathutils::find_distance(prev_pose, current_status_.finalwaypoints.waypoints.at(idx).pose.pose);

    state = current_status_.finalwaypoints.waypoints.at(idx).wpstate.event_state;

    if (state != autoware_msgs::WaypointState::NULLSTATE || distance >= lookahead_distance_)
    {
      break;
    }
    prev_pose = current_status_.finalwaypoints.waypoints.at(idx).pose.pose;
  }
  return state;
}

void DecisionMakerNode::updateFreeAreaState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateLaneAreaState(cstring_t& state_name, int status)
{
  switch (getEventStateFromWaypoint())
  {
    case autoware_msgs::WaypointState::TYPE_EVENT_BUS_STOP:
      tryNextState("on_bus_stop");
      break;
    default:
      tryNextState("on_cruise");
      break;
  }
}

void DecisionMakerNode::updateCruiseState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("received_back_state_waypoint"))
  {
    tryNextState("on_back");
    return;
  }

  if (current_status_.change_flag == enumToInteger<E_ChangeFlags>(E_ChangeFlags::LEFT))
  {
    tryNextState("lane_change_left");
  }
  else if (current_status_.change_flag == enumToInteger<E_ChangeFlags>(E_ChangeFlags::RIGHT))
  {
    tryNextState("lane_change_right");
  }
  else
  {
    switch (getSteeringStateFromWaypoint())
    {
      case autoware_msgs::WaypointState::STR_LEFT:
        tryNextState("on_left_turn");
        break;
      case autoware_msgs::WaypointState::STR_RIGHT:
        tryNextState("on_right_turn");
        break;
      case autoware_msgs::WaypointState::STR_STRAIGHT:
        tryNextState("on_straight");
        break;
      default:
        break;
    }
  }
}

void DecisionMakerNode::entryTurnState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateLeftTurnState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_LEFT);
}
void DecisionMakerNode::updateRightTurnState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_RIGHT);
}
void DecisionMakerNode::updateStraightState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_CLEAR);
}
void DecisionMakerNode::updateBackState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_CLEAR);
}

void DecisionMakerNode::entryLaneChangeState(cstring_t& state_name, int status)
{
  tryNextState("check_target_lane");
}
void DecisionMakerNode::updateLeftLaneChangeState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_LEFT);
}
void DecisionMakerNode::updateCheckLeftLaneState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updateChangeToLeftState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updateRightLaneChangeState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_RIGHT);
}
void DecisionMakerNode::updateCheckRightLaneState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updateChangeToRightState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::updateBusStopState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updatePullInState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_LEFT);
}
void DecisionMakerNode::updatePullOutState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_RIGHT);
}

void DecisionMakerNode::updateParkingState(cstring_t& state_name, int status)
{
  publishLampCmd(E_Lamp::LAMP_HAZARD);
}

}  // namespace decision_maker
