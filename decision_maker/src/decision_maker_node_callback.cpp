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

#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>
#include <utility>

#include <ros/ros.h>

#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/TrafficLight.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace
{
// MISSION COMPLETE FLAG
static constexpr int num_of_set_mission_complete_flag = 3;
}  // namespace

namespace decision_maker
{
void DecisionMakerNode::callbackFromFilteredPoints(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  setEventFlag("received_pointcloud_for_NDT", true);
}

void DecisionMakerNode::callbackFromSimPose(const geometry_msgs::PoseStamped& msg)
{
  ROS_INFO("Received system is going to simulation mode");
  Subs["sim_pose"].shutdown();
}

void DecisionMakerNode::callbackFromStateCmd(const std_msgs::String& msg)
{
  //  ROS_INFO("Received State Command");
  tryNextState(msg.data);
}

void DecisionMakerNode::callbackFromEngage(const std_msgs::Bool& msg)
{
  if (msg.data)
  {
    tryNextState("engage");
  }
}

void DecisionMakerNode::callbackFromStatus(const autoware_msgs::VehicleStatus& msg)
{
  if (msg.drivemode == 1 || msg.steeringmode == 1)
  {
    current_status_.autonomy_engaged = true;
  }
  else
  {
    current_status_.autonomy_engaged = false;
  }
}

void DecisionMakerNode::callbackFromLaneChangeFlag(const std_msgs::Int32& msg)
{
  current_status_.change_flag = msg.data;
}

void DecisionMakerNode::callbackFromConfig(const autoware_config_msgs::ConfigDecisionMaker& msg)
{
  ROS_INFO("Param setted by Runtime Manager");
  auto_mission_reload_ = msg.auto_mission_reload;
  auto_engage_ = msg.auto_engage;
  auto_mission_change_ = msg.auto_mission_change;
  use_fms_ = msg.use_fms;
  lookahead_distance_ = static_cast<double>(msg.num_of_steer_behind);
  mission_change_threshold_dist_ = msg.change_threshold_dist;
  mission_change_threshold_angle_ = msg.change_threshold_angle;
  goal_threshold_dist_ = msg.goal_threshold_dist;
  goal_threshold_vel_ = amathutils::kmph2mps(msg.goal_threshold_vel);
  stopped_vel_ = amathutils::kmph2mps(msg.stopped_vel);
  ignore_map_ = msg.disuse_vector_map;
  sim_mode_ = msg.sim_mode;
  insert_stop_line_wp_ = msg.insert_stop_line_wp;
}

void DecisionMakerNode::callbackFromLightColor(const ros::MessageEvent<autoware_msgs::TrafficLight const>& event)
{
  ROS_WARN("%s is not implemented", __func__);
}

// Insert waypoints inside intersections (crossroads)
void DecisionMakerNode::insertPointWithinCrossRoad(autoware_msgs::LaneArray& lane_array)
{
  for (auto& lane : lane_array.lanes)
  {
    for (auto& wp : lane.waypoints)
    {
      // for each crossroad area
      for (auto& intersect : intersects_)
      {
        // if waypoint exists in crossroad area
        if (intersect.addWaypoint(wp))
        {
          // assign crossroad's aid to waypoint
          wp.wpstate.aid = intersect.area_id;
          // each waypoint should only exist in a single intersection
          break;
        }
      }
    }
  }
}

void DecisionMakerNode::setWaypointStateUsingVectorMap(autoware_msgs::LaneArray& lane_array)
{
  insertPointWithinCrossRoad(lane_array);
  // STR
  for (auto& intersect : intersects_)
  {
    for (auto& laneinArea : intersect.inside_lanes)
    {
      // To straight/left/right recognition by using angle
      // between first-waypoint and end-waypoint in intersection area.
      int angle_deg = (static_cast<int>(std::floor(calcIntersectWayAngle(laneinArea))));  // normalized
      int steering_state;

      if (angle_deg <= ANGLE_LEFT)
        steering_state = autoware_msgs::WaypointState::STR_LEFT;
      else if (angle_deg >= ANGLE_RIGHT)
        steering_state = autoware_msgs::WaypointState::STR_RIGHT;
      else
        steering_state = autoware_msgs::WaypointState::STR_STRAIGHT;

      for (auto& wp_lane : laneinArea.waypoints)
      {
        for (auto& lane : lane_array.lanes)
        {
          for (auto& wp : lane.waypoints)
          {
            if (wp.gid == wp_lane.gid && wp.wpstate.aid == intersect.area_id)
            {
              wp.wpstate.steering_state = steering_state;
            }
          }
        }
      }
    }
  }
  for (auto& lane : lane_array.lanes)
  {
    for (auto& wp : lane.waypoints)
    {
      if (wp.wpstate.steering_state == 0)
      {
        wp.wpstate.steering_state = autoware_msgs::WaypointState::STR_STRAIGHT;
      }
    }
  }

  // Lane Changes - waypoints crossing whitelines
  const std::vector<WhiteLine> whitelines = g_vmap.findByFilter(
    [](const WhiteLine& whitelines) { return true; });  // NOLINT
  for (auto& lane : lane_array.lanes)
  {
    for (size_t wp_idx = 0; wp_idx < lane.waypoints.size() - 1; wp_idx++)
    {
      // Skip waypoints in intersections
      if (lane.waypoints.at(wp_idx).wpstate.aid == 0)
      {
        for (auto& whiteline : whitelines)
        {
          geometry_msgs::Point bp =
            VMPoint2GeoPoint(g_vmap.findByKey(Key<Point>(
              g_vmap.findByKey(Key<Line>(g_vmap.findByKey(Key<WhiteLine>(whiteline.id)).lid)).bpid)));
          geometry_msgs::Point fp =
            VMPoint2GeoPoint(g_vmap.findByKey(Key<Point>(
              g_vmap.findByKey(Key<Line>(g_vmap.findByKey(Key<WhiteLine>(whiteline.id)).lid)).fpid)));
          if (amathutils::isIntersectLine(lane.waypoints.at(wp_idx).pose.pose.position,
                                        lane.waypoints.at(wp_idx + 1).pose.pose.position, bp, fp))
          {
            int steering_state;
            if (amathutils::isPointLeftFromLine(lane.waypoints.at(wp_idx).pose.pose.position, bp, fp) > 0)
            {
              // If waypoint starts from left side of the whiteline, trigger right turn signal
              steering_state = autoware_msgs::WaypointState::STR_RIGHT;
              ROS_INFO("Right turn signal");
            }
            else
            {
              // If waypoint starts from right side of the whiteline, trigger left turn signal
              steering_state = autoware_msgs::WaypointState::STR_LEFT;
              ROS_INFO("Left turn signal");
            }

            ROS_INFO("at: #%zu(%f, %f, %f)", wp_idx,
                      lane.waypoints.at(wp_idx).pose.pose.position.x,
                      lane.waypoints.at(wp_idx).pose.pose.position.y,
                      lane.waypoints.at(wp_idx).pose.pose.position.z);

            // Insert correct turn steering_state at where waypoints crosses the lane
            lane.waypoints.at(wp_idx).wpstate.steering_state = steering_state;
          }
        }
      }
    }
  }

  // Get stoplines associated with stop signs (not traffic lights)
  std::vector<StopLine> stoplines = g_vmap.findByFilter(
    [&](const StopLine& stopline)
    {
      return ((g_vmap.findByKey(Key<RoadSign>(stopline.signid)).type &
        (autoware_msgs::WaypointState::TYPE_STOP | autoware_msgs::WaypointState::TYPE_STOPLINE)) != 0);
    }
  );  // NOLINT

  // Assign appropriate stop_state type to waypoints that intersect stoplines
  for (auto& lane : lane_array.lanes)
  {
    if (lane.waypoints.empty())
    {
      continue;
    }
    for (size_t wp_idx = 0; wp_idx < lane.waypoints.size() - 1; wp_idx++)
    {
      for (auto& stopline : stoplines)
      {
        // fetch each stopline's before and after points
        geometry_msgs::Point bp =
            VMPoint2GeoPoint(g_vmap.findByKey(Key<Point>(g_vmap.findByKey(Key<Line>(stopline.lid)).bpid)));
        geometry_msgs::Point fp =
            VMPoint2GeoPoint(g_vmap.findByKey(Key<Point>(g_vmap.findByKey(Key<Line>(stopline.lid)).fpid)));

        // if waypoints intersect with the above points (waypoints that go through the stoplines)
        if (amathutils::isIntersectLine(lane.waypoints.at(wp_idx).pose.pose.position,
                                        lane.waypoints.at(wp_idx + 1).pose.pose.position, bp, fp))
        {
          geometry_msgs::Point center_point;
          center_point.x = (bp.x * 2 + fp.x) / 3;
          center_point.y = (bp.y * 2 + fp.y) / 3;
          center_point.z = (bp.z + fp.z) / 2;

          // if insert_stop_line_wp param is set to False (default: True)
          // assigns stop_state to existing waypoints closest to the stopline
          if (!insert_stop_line_wp_)
          {
            geometry_msgs::Point intersect_point;
            if (amathutils::getIntersect(lane.waypoints.at(wp_idx).pose.pose.position,
                                         lane.waypoints.at(wp_idx + 1).pose.pose.position, bp, fp, &intersect_point))
            {
              const double dist_front =
                  amathutils::find_distance(intersect_point, lane.waypoints.at(wp_idx + 1).pose.pose.position);
              const double dist_back =
                  amathutils::find_distance(intersect_point, lane.waypoints.at(wp_idx).pose.pose.position);
              int target_wp_idx = wp_idx;
              if (dist_front < dist_back)
              {
                target_wp_idx = wp_idx + 1;
              }
              lane.waypoints.at(target_wp_idx).wpstate.stop_state =
                  g_vmap.findByKey(Key<RoadSign>(stopline.signid)).type;
              ROS_INFO("Change waypoint type to stopline: #%d(%f, %f, %f)\n", target_wp_idx,
                       lane.waypoints.at(target_wp_idx).pose.pose.position.x,
                       lane.waypoints.at(target_wp_idx).pose.pose.position.y,
                       lane.waypoints.at(target_wp_idx).pose.pose.position.z);
            }
          }
          // if insert_stop_line_wp param is set to True (default: True)
          // inserts a new waypoint more accurately where the waypoints and stopline intersects_
          else
          {
            center_point.x = (bp.x + fp.x) / 2;
            center_point.y = (bp.y + fp.y) / 2;
            geometry_msgs::Point interpolation_point =
                amathutils::getNearPtOnLine(center_point, lane.waypoints.at(wp_idx).pose.pose.position,
                                            lane.waypoints.at(wp_idx + 1).pose.pose.position);

            autoware_msgs::Waypoint wp = lane.waypoints.at(wp_idx);
            wp.wpstate.stop_state = g_vmap.findByKey(Key<RoadSign>(stopline.signid)).type;
            wp.pose.pose.position.x = interpolation_point.x;
            wp.pose.pose.position.y = interpolation_point.y;
            wp.pose.pose.position.z =
                (wp.pose.pose.position.z + lane.waypoints.at(wp_idx + 1).pose.pose.position.z) / 2;
            wp.twist.twist.linear.x =
                (wp.twist.twist.linear.x + lane.waypoints.at(wp_idx + 1).twist.twist.linear.x) / 2;

            // label wp with its stopline id
            wp.stop_line_id = stopline.id;

            ROS_INFO("Inserting stopline_interpolation_wp: #%zu(%f, %f, %f)\n", wp_idx + 1, interpolation_point.x,
                     interpolation_point.y, interpolation_point.z);

            lane.waypoints.insert(lane.waypoints.begin() + wp_idx + 1, wp);
            wp_idx++;
          }
        }
      }
    }

    // Mark the last #(num_of_set_mission_complete_flag) of waypoints with MISSION COMPLETE FLAG
    size_t wp_idx = lane.waypoints.size();
    if (wp_idx > num_of_set_mission_complete_flag)
    {
      wp_idx = num_of_set_mission_complete_flag;
    }
    for (unsigned int counter = 0; counter <= wp_idx; counter++)
    {
      lane.waypoints.at(--wp_idx).wpstate.event_state = autoware_msgs::WaypointState::TYPE_EVENT_GOAL;
    }
  }
}

void DecisionMakerNode::setWaypointStateUsingLanelet2Map(autoware_msgs::LaneArray& lane_array)
{
  std::map<int, lanelet::Id> wp2laneletid;
  lanelet::utils::matchWaypointAndLanelet(lanelet_map_, routing_graph_, lane_array, &wp2laneletid);

  for (const auto& item : wp2laneletid)
  {
    ROS_DEBUG_STREAM("matched waypoint_gid and lanelet_id: " << item.first << " " << item.second);
  }

  insertPointWithinCrossRoad(lane_array);

  // Add steering state attribute from lanelet map.
  for (auto& lane : lane_array.lanes)
  {
    for (auto& wp : lane.waypoints)
    {
      // Only assign wp steering state if not already assigned
      if (wp.wpstate.steering_state != autoware_msgs::WaypointState::NULLSTATE)
      {
        continue;
      }

      // default is straight
      int steering_state = autoware_msgs::WaypointState::STR_STRAIGHT;
      if (wp2laneletid.find(wp.gid) != wp2laneletid.end())
      {
        int lanelet_id = wp2laneletid.at(wp.gid);
        const auto& lanelet = lanelet_map_->laneletLayer.get(lanelet_id);
        std::string direction = lanelet.attributeOr("turn_direction", "straight");
        if (direction == "right")
        {
          steering_state = autoware_msgs::WaypointState::STR_RIGHT;
        }
        if (direction == "left")
        {
          steering_state = autoware_msgs::WaypointState::STR_LEFT;
        }
      }
      wp.wpstate.steering_state = steering_state;
    }
  }

  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_);

  // Lane Changes - waypoints crossing whitelines
  lanelet::ConstLanelets roads = lanelet::utils::query::roadLanelets(all_lanelets);

  for (auto& lane : lane_array.lanes)
  {
    for (size_t wp_idx = 0; wp_idx < lane.waypoints.size() - 1; wp_idx++)
    {
      // Skip waypoints in intersections
      if (lane.waypoints.at(wp_idx).wpstate.aid == 0)
      {
        for (const auto& road : roads)
        {
          // skip lanelets that belong to an intersection
          if (road.attributeOr("turn_direction", "empty") != "empty")
          {
            continue;
          }
          // skip lanelets with fewer than two points
          if (road.leftBound().size() < 2 | road.rightBound().size() < 2)
          {
            continue;
          }

          std::vector<std::pair<int, geometry_msgs::Point>> pts;

          for (const auto& pt : road.leftBound())
          {
            // label 1 for leftBound points
            pts.push_back(make_pair(1, lanelet::utils::conversion::toGeomMsgPt(pt)));
          }
          for (const auto& pt : road.rightBound())
          {
            // label 2 for leftBound points
            pts.push_back(make_pair(2, lanelet::utils::conversion::toGeomMsgPt(pt)));
          }

          std::pair<int, geometry_msgs::Point> prev_pt;
          for (auto& pt : pts)
          {
            // skip first to load first pair for isIntersectLine function
            if (prev_pt.first == 0)
            {
              prev_pt = pt;
              continue;
            }
            // pairs only if the two points belong to the same bound
            if (pt.first == prev_pt.first)
            {
              if (amathutils::isIntersectLine(
                  lane.waypoints.at(wp_idx).pose.pose.position, lane.waypoints.at(wp_idx + 1).pose.pose.position,
                  prev_pt.second, pt.second))
              {
                int steering_state;
                if (amathutils::isPointLeftFromLine(lane.waypoints.at(wp_idx).pose.pose.position,
                                                    prev_pt.second, pt.second) > 0)
                {
                  // If waypoint starts from left side of the whiteline, trigger right turn signal
                  steering_state = autoware_msgs::WaypointState::STR_RIGHT;
                  ROS_INFO("Right turn signal");
                }
                else
                {
                  // If waypoint starts from right side of the whiteline, trigger left turn signal
                  steering_state = autoware_msgs::WaypointState::STR_LEFT;
                  ROS_INFO("Left turn signal");
                }
                ROS_INFO("From: #%zu(%f, %f, %f)", wp_idx,
                          lane.waypoints.at(wp_idx).pose.pose.position.x,
                          lane.waypoints.at(wp_idx).pose.pose.position.y,
                          lane.waypoints.at(wp_idx).pose.pose.position.z);

                // Insert correct turn steering_state at where waypoints crosses the lane
                lane.waypoints.at(wp_idx).wpstate.steering_state = steering_state;
              }
            }
            prev_pt = pt;
          }
        }
      }
    }
  }

  // Get stop lines associated with stop signs (not traffic lights)
  lanelet::ConstLineStrings3d stoplines = lanelet::utils::query::getStopSignStopLines(all_lanelets);

  for (auto& lane : lane_array.lanes)
  {
    if (lane.waypoints.empty())
    {
      continue;
    }
    for (size_t wp_idx = 0; wp_idx < lane.waypoints.size() - 1; wp_idx++)
    {
      autoware_msgs::Waypoint wp = lane.waypoints.at(wp_idx);

      for (const auto& stopline : stoplines)
      {
        // skip invalid stopline (line without point)
        if (stopline.empty())
          continue;

        // check if lanelet is bidirectional
        bool is_bidirectional = false;

        if (wp2laneletid.find(wp.gid) != wp2laneletid.end())
        {
          int lanelet_id = wp2laneletid.at(lane.waypoints.at(wp_idx).gid);
          lanelet::ConstLanelet lanelet = lanelet_map_->laneletLayer.get(lanelet_id);
          is_bidirectional = lanelet.attributeOr("one_way", false);
        }

        const geometry_msgs::Point bp = lanelet::utils::conversion::toGeomMsgPt(stopline.front());
        const geometry_msgs::Point fp = lanelet::utils::conversion::toGeomMsgPt(stopline.back());
        if (amathutils::isIntersectLine(lane.waypoints.at(wp_idx).pose.pose.position,
                                        lane.waypoints.at(wp_idx + 1).pose.pose.position, bp, fp))
        {
          // direction of stopline would be important only if the lanelet is
          // bidrectional
          if (!is_bidirectional ||
              amathutils::isPointLeftFromLine(bp, lane.waypoints.at(wp_idx).pose.pose.position,
                                              lane.waypoints.at(wp_idx + 1).pose.pose.position) >= 0)
          {
            geometry_msgs::Point center_point;
            center_point.x = (bp.x + fp.x) / 2;
            center_point.y = (bp.y + fp.y) / 2;
            center_point.z = (bp.z + fp.z) / 2;

            // if insert_stop_line_wp param is set to False (default: True)
            // assigns stop_state to existing waypoints closest to the stopline
            if (!insert_stop_line_wp_)
            {
              geometry_msgs::Point intersect_point;
              if (amathutils::getIntersect(lane.waypoints.at(wp_idx).pose.pose.position,
                                           lane.waypoints.at(wp_idx + 1).pose.pose.position, bp, fp, &intersect_point))
              {
                const double dist_front =
                    amathutils::find_distance(intersect_point, lane.waypoints.at(wp_idx + 1).pose.pose.position);
                const double dist_back =
                    amathutils::find_distance(intersect_point, lane.waypoints.at(wp_idx).pose.pose.position);
                int target_wp_idx = wp_idx;
                if (dist_front < dist_back)
                  target_wp_idx = wp_idx + 1;
                lane.waypoints.at(target_wp_idx).wpstate.stop_state = autoware_msgs::WaypointState::TYPE_STOPLINE;
                ROS_INFO("Change waypoint type to stopline: #%d(%f, %f, %f)\n", target_wp_idx,
                         lane.waypoints.at(target_wp_idx).pose.pose.position.x,
                         lane.waypoints.at(target_wp_idx).pose.pose.position.y,
                         lane.waypoints.at(target_wp_idx).pose.pose.position.z);
              }
            }
            // if insert_stop_line_wp param is set to True (default: True)
            // inserts a new waypoint more accurately where the waypoints and stopline intersects_
            else
            {
              center_point.x = (bp.x + fp.x) / 2;
              center_point.y = (bp.y + fp.y) / 2;
              geometry_msgs::Point interpolation_point =
                  amathutils::getNearPtOnLine(center_point, lane.waypoints.at(wp_idx).pose.pose.position,
                                              lane.waypoints.at(wp_idx + 1).pose.pose.position);

              autoware_msgs::Waypoint wp = lane.waypoints.at(wp_idx);
              wp.wpstate.stop_state = autoware_msgs::WaypointState::TYPE_STOPLINE;
              wp.pose.pose.position.x = interpolation_point.x;
              wp.pose.pose.position.y = interpolation_point.y;
              wp.pose.pose.position.z =
                  (wp.pose.pose.position.z + lane.waypoints.at(wp_idx + 1).pose.pose.position.z) / 2;
              wp.twist.twist.linear.x =
                  (wp.twist.twist.linear.x + lane.waypoints.at(wp_idx + 1).twist.twist.linear.x) / 2;

              // label wp with its stopline id
              wp.stop_line_id = stopline.id();

              ROS_INFO("Inserting stopline_interpolation_wp: #%zu(%f, %f, %f)\n", wp_idx + 1, interpolation_point.x,
                       interpolation_point.y, interpolation_point.z);

              lane.waypoints.insert(lane.waypoints.begin() + wp_idx + 1, wp);
              wp_idx++;
            }
          }
        }
      }
    }

    // Mark the last #(num_of_set_mission_complete_flag) of waypoints with MISSION COMPLETE FLAG
    size_t wp_idx = lane.waypoints.size();
    if (wp_idx > num_of_set_mission_complete_flag)
    {
      wp_idx = num_of_set_mission_complete_flag;
    }
    for (unsigned int counter = 0; counter <= wp_idx; counter++)
    {
      lane.waypoints.at(--wp_idx).wpstate.event_state = autoware_msgs::WaypointState::TYPE_EVENT_GOAL;
    }
  }
}

bool DecisionMakerNode::drivingMissionCheck()
{
  publishOperatorHelpMessage("Received new mission, checking now...");

  auto ret = prepareActiveLaneArray();
  if (ret.first > mission_change_threshold_dist_ || ret.second > mission_change_threshold_angle_)
  {
    return false;
  }
  else
  {
    current_status_.active_lane_array = current_status_.based_lane_array;
    Pubs["lane_waypoints_array"].publish(current_status_.active_lane_array);
    if (!isSubscriberRegistered("final_waypoints"))
    {
      Subs["final_waypoints"] =
          nh_.subscribe("final_waypoints", 100, &DecisionMakerNode::callbackFromFinalWaypoint, this);
    }

    return true;
  }
}

// for based waypoint
void DecisionMakerNode::callbackFromLaneWaypoint(const autoware_msgs::LaneArray& msg)
{
  ROS_INFO("[%s]:LoadedWaypointLaneArray\n", __func__);

  current_status_.based_lane_array = msg;
  setEventFlag("received_based_lane_waypoint", true);
}

void DecisionMakerNode::callbackFromFinalWaypoint(const autoware_msgs::Lane& msg)
{
  current_status_.finalwaypoints = msg;
  setEventFlag("received_finalwaypoints", true);
}

void DecisionMakerNode::callbackFromClosestWaypoint(const std_msgs::Int32& msg)
{
  current_status_.closest_waypoint = msg.data;

  if (current_status_.current_intersection_ptr == nullptr)
    return;

  if (current_status_.current_intersection_ptr->inside_waypoints.size() == 0)
    return;

  // if vehicle exits out of the intersection, reset curr_stopped_idx
  if (msg.data > current_status_.current_intersection_ptr->inside_waypoints.back().gid)
  {
     current_status_.curr_stopped_idx = -1;
  }
  // if vehicle backs up past the stopline, reset curr_stopped_idx
  if (msg.data - current_status_.current_intersection_ptr->inside_waypoints.front().gid > stopline_reset_count_)
  {
    current_status_.curr_stopped_idx = -1;
  }
}

void DecisionMakerNode::callbackFromCurrentPose(const geometry_msgs::PoseStamped& msg)
{
  current_status_.pose = msg.pose;
}

void DecisionMakerNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStamped& msg)
{
  current_status_.velocity = msg.twist.linear.x;
}

void DecisionMakerNode::callbackFromObstacleWaypoint(const std_msgs::Int32& msg)
{
  current_status_.obstacle_waypoint = msg.data;
}

void DecisionMakerNode::callbackFromStoplineWaypoint(const std_msgs::Int32& msg)
{
  current_status_.stopline_waypoint = msg.data;
}

void DecisionMakerNode::callbackFromStopOrder(const std_msgs::Int32& msg)
{
  autoware_msgs::VehicleLocation pub_msg;
  pub_msg.header.stamp = ros::Time::now();
  pub_msg.lane_array_id = current_status_.active_lane_array.id;
  pub_msg.waypoint_index = -1;

  if (current_status_.closest_waypoint < msg.data &&
      msg.data < current_status_.active_lane_array.lanes.back().waypoints.back().gid)
  {
    current_status_.prev_ordered_idx = current_status_.ordered_stop_idx;
    current_status_.ordered_stop_idx = msg.data;
    pub_msg.waypoint_index = msg.data;
  }
  else
  {
    current_status_.ordered_stop_idx = -1;
  }

  Pubs["stop_cmd_location"].publish(pub_msg);
}

void DecisionMakerNode::callbackFromLanelet2Map(const autoware_lanelet2_msgs::MapBin::ConstPtr& msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
      lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  routing_graph_ = lanelet::routing::RoutingGraph::build(*lanelet_map_, *traffic_rules);
  setEventFlag("lanelet2_map_loaded", true);
  ROS_INFO("Loaded lanelet2 map");
}

void DecisionMakerNode::callbackFromDetection(const autoware_msgs::DetectedObjectArray& msg)
{
  // don't check until current_intersection is found
  if (current_status_.current_intersection_ptr == nullptr)
    return;

  // don't check until ego reaches the intersection
  if (fabs(current_status_.velocity) > stopped_vel_)
    return;

  if (current_status_.stopline_waypoint != -1)
  {
    if (current_status_.current_intersection_ptr->isObjectInsideStopAreas(msg, stopline_init_phase2_flag_))
    {
      current_status_.stopline_safety_timer = ros::Time::now();
    }
    if (!stopline_init_phase2_flag_)
    {
      stopline_init_phase2_flag_ = true;
    }
  }
}

}  // namespace decision_maker
