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
void DecisionMakerNode::entryMissionInitState(cstring_t& state_name, int status)
{
}
void DecisionMakerNode::updateMissionInitState(cstring_t& state_name, int status)
{
  if (!use_fms_)
  {
    tryNextState("state_mission_initialized");
  }
}

void DecisionMakerNode::entryWaitOrderState(cstring_t& state_name, int status)
{
  publishOperatorHelpMessage("Please load mission order (waypoints).");
  setEventFlag("received_based_lane_waypoint", false);
  if (!isSubscriberRegistered("lane_waypoints_array"))
  {
    Subs["lane_waypoints_array"] =
        nh_.subscribe(TPNAME_BASED_LANE_WAYPOINTS_ARRAY, 1, &DecisionMakerNode::callbackFromLaneWaypoint, this);
  }
}

void DecisionMakerNode::updateWaitOrderState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("received_based_lane_waypoint"))
  {
    setEventFlag("received_based_lane_waypoint", false);
    tryNextState("received_mission_order");
  }
}
void DecisionMakerNode::exitWaitOrderState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::entryMissionCheckState(cstring_t& state_name, int status)
{
  publishOperatorHelpMessage("Received mission, checking now...");
  setEventFlag("received_back_state_waypoint", false);

  prepareActiveLaneArray();

  current_status_.active_lane_array = current_status_.based_lane_array;
  Pubs["lane_waypoints_array"].publish(current_status_.active_lane_array);
  if (!isSubscriberRegistered("final_waypoints"))
  {
    Subs["final_waypoints"] =
        nh_.subscribe("final_waypoints", 1, &DecisionMakerNode::callbackFromFinalWaypoint, this);
  }
  if (!isSubscriberRegistered("stop_order_idx"))
  {
    Subs["stop_order_idx"] =
      nh_.subscribe("state/stop_order_wpidx", 1, &DecisionMakerNode::callbackFromStopOrder, this);
  }
}

void DecisionMakerNode::updateMissionCheckState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("received_finalwaypoints") && current_status_.closest_waypoint != -1)
  {
    if (current_status_.finalwaypoints.waypoints.size() < 5)
      publishOperatorHelpMessage(
        "Finalwaypoints is too short.If you want to Engage,\n"
        "please publish \"mission_is_compatible\" key by \"state_cmd\" topic.");
    else
      tryNextState("mission_is_compatible");
  }
  else
  {
    if (current_status_.closest_waypoint == -1)
      publishOperatorHelpMessage("[ERROR]Couldn't received \"closest_waypoint\" or its value is -1.");
    if (!isEventFlagTrue("received_finalwaypoints"))
      publishOperatorHelpMessage("[ERROR]Couldn't received \"final_waypoints\".");
  }
}

void DecisionMakerNode::entryMissionAbortedState(cstring_t& state_name, int status)
{
  tryNextState("operation_end");
}
void DecisionMakerNode::updateMissionAbortedState(cstring_t& state_name, int status)
{
  if (!use_fms_)
  {
    sleep(1);
    tryNextState("goto_wait_order");
    return;
  }
}

void DecisionMakerNode::entryDriveReadyState(cstring_t& state_name, int status)
{
  publishOperatorHelpMessage("Please order to engage");
}

void DecisionMakerNode::updateDriveReadyState(cstring_t& state_name, int status)
{
  if (!use_fms_ && auto_engage_)
  {
    tryNextState("engage");
  }
}

void DecisionMakerNode::entryDrivingState(cstring_t& state_name, int status)
{
  setEventFlag("received_based_lane_waypoint", false);

  tryNextState("operation_start");
}
void DecisionMakerNode::updateDrivingState(cstring_t& state_name, int status)
{
  if (!use_fms_ && auto_mission_change_ && isEventFlagTrue("received_based_lane_waypoint"))
  {
    tryNextState("request_mission_change");
  }
}
void DecisionMakerNode::exitDrivingState(cstring_t& state_name, int status)
{
}

void DecisionMakerNode::entryDrivingMissionChangeState(cstring_t& state_name, int status)
{
  if (!auto_mission_change_)
    setEventFlag("received_based_lane_waypoint", false);
}

void DecisionMakerNode::updateDrivingMissionChangeState(cstring_t& state_name, int status)
{
  if (isEventFlagTrue("received_based_lane_waypoint"))
  {
    setEventFlag("received_based_lane_waypoint", false);
    if (!drivingMissionCheck())
    {
      publishOperatorHelpMessage("Failed to change the mission.");
      tryNextState("mission_is_conflicting");
      return;
    }
    else
    {
      publishOperatorHelpMessage("Mission change succeeded.");
      tryNextState("mission_is_compatible");
      return;
    }
  }
}

void DecisionMakerNode::updateMissionChangeSucceededState(cstring_t& state_name, int status)
{
  if (!use_fms_)
  {
    sleep(1);
    tryNextState("return_to_driving");
  }
}
void DecisionMakerNode::updateMissionChangeFailedState(cstring_t& state_name, int status)
{
  if (!use_fms_)
  {
    sleep(1);
    tryNextState("return_to_driving");
  }
}

void DecisionMakerNode::entryMissionCompleteState(cstring_t& state_name, int status)
{
  setEventFlag("received_based_lane_waypoint", false);

  if (!use_fms_ && auto_mission_reload_)
    tryNextState("mission_reloaded");
  else
    tryNextState("operation_end");
}
void DecisionMakerNode::updateMissionCompleteState(cstring_t& state_name, int status)
{
  setEventFlag("received_based_lane_waypoint", false);
  if (!use_fms_)
  {
    if (auto_mission_reload_)
    {
      publishOperatorHelpMessage("Reload mission.");
      tryNextState("re_enter_mission");
      return;
    }
    else
    {
      sleep(1);
      tryNextState("goto_wait_order");
      return;
    }
  }
}
}  // namespace decision_maker
