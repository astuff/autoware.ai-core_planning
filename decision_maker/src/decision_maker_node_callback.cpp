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
  param_num_of_steer_behind_ = msg.num_of_steer_behind;
  change_threshold_dist_ = msg.change_threshold_dist;
  change_threshold_angle_ = msg.change_threshold_angle;
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

void DecisionMakerNode::insertPointWithinCrossRoad(const std::vector<CrossRoadArea>& _intersects,
                                                   autoware_msgs::LaneArray& lane_array)
{
  for (auto& lane : lane_array.lanes)
  {
    for (auto& wp : lane.waypoints)
    {
      geometry_msgs::Point pp;
      pp.x = wp.pose.pose.position.x;
      pp.y = wp.pose.pose.position.y;
      pp.z = wp.pose.pose.position.z;

      for (auto& area : intersects)
      {
        if (CrossRoadArea::isInsideArea(&area, pp))
        {
          // area's
          if (area.insideLanes.empty() || wp.gid != area.insideLanes.back().waypoints.back().gid + 1)
          {
            autoware_msgs::Lane nlane;
            area.insideLanes.push_back(nlane);
            area.bbox.pose.orientation = wp.pose.pose.orientation;
          }
          area.insideLanes.back().waypoints.push_back(wp);
          area.insideWaypoint_points.push_back(pp);  // geometry_msgs::point
          // area.insideLanes.Waypoints.push_back(wp);//autoware_msgs::Waypoint
          // lane's wp
          wp.wpstate.aid = area.area_id;
        }
      }
    }
  }
}

void DecisionMakerNode::setWaypointStateUsingVectorMap(autoware_msgs::LaneArray& lane_array)
{
  insertPointWithinCrossRoad(intersects, lane_array);
  // STR
  for (auto& area : intersects)
  {
    for (auto& laneinArea : area.insideLanes)
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
            if (wp.gid == wp_lane.gid && wp.wpstate.aid == area.area_id)
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
            unsigned int i;
            // turn signal should trigger 30 meters before the lane change
            for (i = 1; i < wp_idx; i++)
            {
              double dist = amathutils::find_distance(lane.waypoints.at(wp_idx).pose.pose.position,
                                                lane.waypoints.at(wp_idx-i).pose.pose.position);
              if (dist >= distance_before_lane_change_signal_)
                break;
            }

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

            ROS_INFO("From: #%zu(%f, %f, %f)", wp_idx - i,
                      lane.waypoints.at(wp_idx - i).pose.pose.position.x,
                      lane.waypoints.at(wp_idx - i).pose.pose.position.y,
                      lane.waypoints.at(wp_idx - i).pose.pose.position.z);
            ROS_INFO("To:   #%zu(%f, %f, %f)", wp_idx + 1,
                      lane.waypoints.at(wp_idx + 1).pose.pose.position.x,
                      lane.waypoints.at(wp_idx + 1).pose.pose.position.y,
                      lane.waypoints.at(wp_idx + 1).pose.pose.position.z);

            // Insert right turn steering_state up to where waypoints crosses the lane
            for (unsigned int j = 0; j <= i + 1; j++)
            {
              lane.waypoints.at(wp_idx - j + 1).wpstate.steering_state = steering_state;
            }
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
              double dist_front =
                  amathutils::find_distance(intersect_point, lane.waypoints.at(wp_idx + 1).pose.pose.position);
              double dist_back =
                  amathutils::find_distance(intersect_point, lane.waypoints.at(wp_idx).pose.pose.position);
              int target_wp_idx = wp_idx;
              if (dist_front < dist_back)
                target_wp_idx = wp_idx + 1;
              lane.waypoints.at(target_wp_idx).wpstate.stop_state =
                  g_vmap.findByKey(Key<RoadSign>(stopline.signid)).type;
              ROS_INFO("Change waypoint type to stopline: #%d(%f, %f, %f)\n", target_wp_idx,
                       lane.waypoints.at(target_wp_idx).pose.pose.position.x,
                       lane.waypoints.at(target_wp_idx).pose.pose.position.y,
                       lane.waypoints.at(target_wp_idx).pose.pose.position.z);
            }
          }
          // if insert_stop_line_wp param is set to True (default: True)
          // inserts a new waypoint more accurately where the waypoints and stopline intersects
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

            ROS_INFO("Inserting stopline_interpolation_wp: #%zu(%f, %f, %f)\n", wp_idx + 1, interpolation_point.x,
                     interpolation_point.y, interpolation_point.z);

            lane.waypoints.insert(lane.waypoints.begin() + wp_idx + 1, wp);
            wp_idx++;
          }
        }
      }
    }

    size_t wp_idx = lane.waypoints.size();
    for (unsigned int counter = 0;
         counter <= (wp_idx <= num_of_set_mission_complete_flag ? wp_idx : num_of_set_mission_complete_flag); counter++)
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

  insertPointWithinCrossRoad(intersects, lane_array);

  // Add steering state attribute from lanelet map.
  for (auto& lane : lane_array.lanes)
  {
    for (auto& wp : lane.waypoints)
    {
      // default is straight
      int steering_state = autoware_msgs::WaypointState::STR_STRAIGHT;
      if (wp2laneletid.find(wp.gid) != wp2laneletid.end())
      {
        int lanelet_id = wp2laneletid.at(wp.gid);
        lanelet::ConstLanelet lanelet = lanelet_map_->laneletLayer.get(lanelet_id);
        std::string direction = lanelet.attributeOr("turn_direction", "straight");
        if (direction.compare("right") == 0)
        {
          steering_state = autoware_msgs::WaypointState::STR_RIGHT;
        }
        if (direction.compare("left") == 0)
        {
          steering_state = autoware_msgs::WaypointState::STR_LEFT;
        }
      }
      wp.wpstate.steering_state = steering_state;
    }
  }

  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_);

  // Get stop lines associated with stop signs (not traffic lights)
  lanelet::ConstLineStrings3d stoplines = lanelet::utils::query::getStopSignStopLines(all_lanelets, stop_sign_id_);

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

            geometry_msgs::Point interpolation_point =
                amathutils::getNearPtOnLine(center_point, lane.waypoints.at(wp_idx).pose.pose.position,
                                            lane.waypoints.at(wp_idx + 1).pose.pose.position);

            wp.wpstate.stop_state = autoware_msgs::WaypointState::TYPE_STOPLINE;  // STOP
            wp.pose.pose.position.x = interpolation_point.x;
            wp.pose.pose.position.y = interpolation_point.y;
            wp.pose.pose.position.z =
                (wp.pose.pose.position.z + lane.waypoints.at(wp_idx + 1).pose.pose.position.z) / 2;
            wp.twist.twist.linear.x =
                (wp.twist.twist.linear.x + lane.waypoints.at(wp_idx + 1).twist.twist.linear.x) / 2;

            ROS_INFO("Inserting stopline_interpolation_wp: #%zu(%f, %f, %f)\n", wp_idx + 1, interpolation_point.x,
                     interpolation_point.y, interpolation_point.z);

            lane.waypoints.insert(lane.waypoints.begin() + wp_idx + 1, wp);
            wp_idx++;
          }
        }
      }
    }

    size_t wp_idx = lane.waypoints.size();
    for (unsigned int counter = 0;
         counter <= (wp_idx <= num_of_set_mission_complete_flag ? wp_idx : num_of_set_mission_complete_flag); counter++)
    {
      lane.waypoints.at(--wp_idx).wpstate.event_state = autoware_msgs::WaypointState::TYPE_EVENT_GOAL;
    }
  }
}

bool DecisionMakerNode::drivingMissionCheck()
{
  publishOperatorHelpMessage("Received new mission, checking now...");
  setEventFlag("received_back_state_waypoint", false);

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
        wp.wpstate.stop_state = autoware_msgs::WaypointState::NULLSTATE;
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

  // waypoint-state set and insert interpolation waypoint for stopline
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

  // reindexing and calculate new closest_waypoint distance
  gid = 0;
  double min_dist = 100;
  geometry_msgs::Pose nearest_wp_pose;
  for (auto& lane : current_status_.based_lane_array.lanes)
  {
    int lid = 0;
    std::vector<double> dist_vec;
    dist_vec.reserve(lane.waypoints.size());
    for (auto& wp : lane.waypoints)
    {
      wp.gid = gid++;
      wp.lid = lid++;
      double dst = amathutils::find_distance(current_status_.pose.position, wp.pose.pose.position);
      if (min_dist > dst)
      {
        min_dist = dst;
        nearest_wp_pose = wp.pose.pose;
      }
    }
  }

  const double angle_diff_degree = std::abs(amathutils::calcPosesAngleDiffDeg(current_status_.pose, nearest_wp_pose));
  if (min_dist > change_threshold_dist_ || angle_diff_degree > change_threshold_angle_)
  {
    return false;
  }
  else
  {
    current_status_.using_lane_array = current_status_.based_lane_array;
    Pubs["lane_waypoints_array"].publish(current_status_.using_lane_array);
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
  pub_msg.lane_array_id = current_status_.using_lane_array.id;
  pub_msg.waypoint_index = -1;

  if (current_status_.closest_waypoint < msg.data &&
      msg.data < current_status_.using_lane_array.lanes.back().waypoints.back().gid)
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

}  // namespace decision_maker
