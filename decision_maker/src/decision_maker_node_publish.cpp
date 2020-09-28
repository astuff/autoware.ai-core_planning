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

#include <string>
#include <vector>

namespace decision_maker
{
void DecisionMakerNode::publishLampCmd(const E_Lamp& status)
{
  autoware_msgs::LampCmd lamp_msg;
  lamp_msg.header.stamp = ros::Time::now();

  switch (status)
  {
    case E_Lamp::LAMP_LEFT:
      lamp_msg.l = LAMP_ON;
      lamp_msg.r = LAMP_OFF;
      break;
    case E_Lamp::LAMP_RIGHT:
      lamp_msg.l = LAMP_OFF;
      lamp_msg.r = LAMP_ON;
      break;
    case E_Lamp::LAMP_HAZARD:
      lamp_msg.l = LAMP_ON;
      lamp_msg.r = LAMP_ON;
      break;
    case E_Lamp::LAMP_EMPTY:
    default:
      lamp_msg.l = LAMP_OFF;
      lamp_msg.r = LAMP_OFF;
      break;
  }
  Pubs["lamp_cmd"].publish(lamp_msg);
}

jsk_rviz_plugins::OverlayText createOverlayText(cstring_t& data, const int column)
{
  jsk_rviz_plugins::OverlayText ret;

  // message setup
  ret.width = 500;
  ret.height = 500;
  ret.top = 10 + (column * 500);
  ret.left = 10;
  ret.bg_color.r = 0;
  ret.bg_color.g = 0;
  ret.bg_color.b = 0;
  ret.bg_color.a = 0.8;

  ret.line_width = 2;
  ret.text_size = 9;
  ret.font = "DejaVu Sans Mono";
  ret.fg_color.r = 1.0;
  ret.fg_color.g = 1.0;
  ret.fg_color.b = 0.5;
  ret.fg_color.a = 0.9;

  ret.text = data;

  return ret;
}

void DecisionMakerNode::publishOperatorHelpMessage(const cstring_t& message)
{
  static std::vector<std::string> msg_log;
  static const size_t log_size = 10;

  msg_log.push_back(message);

  if (msg_log.size() >= log_size)
  {
    msg_log.erase(msg_log.begin());
  }

  std::string joined_msg;
  for (const auto& i : msg_log)
  {
    joined_msg += "> " + i + "\n";
  }
  Pubs["operator_help_text"].publish(createOverlayText(joined_msg, 0));
}

void DecisionMakerNode::update_msgs(void)
{
  if (ctx_vehicle && ctx_mission && ctx_behavior && ctx_motion)
  {
    static std::string text_vehicle_state, text_mission_state, text_behavior_state, text_motion_state;
    text_vehicle_state = ctx_vehicle->getStateText();
    text_mission_state = ctx_mission->getStateText();
    text_behavior_state = ctx_behavior->getStateText();
    text_motion_state = ctx_motion->getStateText();

    static std_msgs::String state_msg;
    state_msg.data = text_vehicle_state + text_mission_state + text_behavior_state + text_motion_state;
    Pubs["state"].publish(state_msg);

    static std::string overlay_text;
    overlay_text = "> Vehicle:\n" + text_vehicle_state +
                   "\n> Mission:\n" + text_mission_state +
                   "\n> Behavior:\n" + text_behavior_state +
                   "\n> Motion:\n" + text_motion_state;
    Pubs["state_overlay"].publish(createOverlayText(overlay_text, 1));

    static autoware_msgs::State state_array_msg;
    state_array_msg.header.stamp = ros::Time::now();
    state_array_msg.vehicle_state = text_vehicle_state;
    state_array_msg.mission_state = text_mission_state;
    state_array_msg.behavior_state = text_behavior_state;
    state_array_msg.motion_state = text_motion_state;
    Pubs["state_msg"].publish(state_array_msg);

    static std_msgs::String transition_msg;
    transition_msg.data = ctx_vehicle->getAvailableTransition() + "\n" + ctx_mission->getAvailableTransition() + "\n" +
                          ctx_behavior->getAvailableTransition() + "\n" + ctx_motion->getAvailableTransition();

    Pubs["available_transition"].publish(transition_msg);
  }
  else
  {
    std::cerr << "ctx is not found " << std::endl;
  }
}

void DecisionMakerNode::publishStoplineWaypointIdx(const int wp_idx)
{
  std_msgs::Int32 msg;
  msg.data = wp_idx;
  Pubs["state/stopline_wpidx"].publish(msg);
}

void DecisionMakerNode::displayStopZone()
{
  if (current_status_.current_intersection_ptr == nullptr)
    return;

  for (const auto& stop_area : current_status_.current_intersection_ptr->stop_areas)
  {
    // only display unsafe areas
    if (stop_area.is_safe != 0)
    {
      continue;
    }
    for (const auto& pt : stop_area.roi_points)
    {
      stop_zone_marker_.points.push_back(pt);
    }
  }
  Pubs["stop_zone"].publish(stop_zone_marker_);
  stop_zone_marker_.points.clear();
}

}  // namespace decision_maker
