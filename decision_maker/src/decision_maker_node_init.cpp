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

#include <algorithm>
#include <vector>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>

namespace decision_maker
{

void DecisionMakerNode::setupStateCallback(void)
{
  /*INIT*/
  /*** state vehicle ***/
  ctx_vehicle->setCallback(
    state_machine::CallbackType::ENTRY, "Init",
    std::bind(&DecisionMakerNode::entryInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::UPDATE, "Init",
    std::bind(&DecisionMakerNode::updateInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::ENTRY, "SensorInit",
    std::bind(&DecisionMakerNode::entrySensorInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::UPDATE, "SensorInit",
    std::bind(&DecisionMakerNode::updateSensorInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::ENTRY, "LocalizationInit",
    std::bind(&DecisionMakerNode::entryLocalizationInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::UPDATE, "LocalizationInit",
    std::bind(&DecisionMakerNode::updateLocalizationInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::ENTRY, "PlanningInit",
    std::bind(&DecisionMakerNode::entryPlanningInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::UPDATE, "PlanningInit",
    std::bind(&DecisionMakerNode::updatePlanningInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::ENTRY, "VehicleInit",
    std::bind(&DecisionMakerNode::entryVehicleInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::UPDATE, "VehicleInit",
    std::bind(&DecisionMakerNode::updateVehicleInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::ENTRY, "VehicleReady",
    std::bind(&DecisionMakerNode::entryVehicleReadyState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::UPDATE, "VehicleReady",
    std::bind(&DecisionMakerNode::updateVehicleReadyState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::UPDATE, "BatteryCharging",
    std::bind(&DecisionMakerNode::updateBatteryChargingState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::ENTRY, "VehicleEmergency",
    std::bind(&DecisionMakerNode::entryVehicleEmergencyState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(
    state_machine::CallbackType::UPDATE, "VehicleEmergency",
    std::bind(&DecisionMakerNode::updateVehicleEmergencyState, this, std::placeholders::_1, 0));

  /*** state mission ***/
  ctx_mission->setCallback(
    state_machine::CallbackType::ENTRY, "MissionInit",
    std::bind(&DecisionMakerNode::entryMissionInitState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::UPDATE, "MissionInit",
    std::bind(&DecisionMakerNode::updateMissionInitState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::ENTRY, "WaitOrder",
    std::bind(&DecisionMakerNode::entryWaitOrderState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::UPDATE, "WaitOrder",
    std::bind(&DecisionMakerNode::updateWaitOrderState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::EXIT, "WaitOrder",
    std::bind(&DecisionMakerNode::exitWaitOrderState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::ENTRY, "MissionCheck",
    std::bind(&DecisionMakerNode::entryMissionCheckState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::UPDATE, "MissionCheck",
    std::bind(&DecisionMakerNode::updateMissionCheckState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::ENTRY, "DriveReady",
    std::bind(&DecisionMakerNode::entryDriveReadyState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::UPDATE, "DriveReady",
    std::bind(&DecisionMakerNode::updateDriveReadyState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::ENTRY, "Driving",
    std::bind(&DecisionMakerNode::entryDrivingState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::UPDATE, "Driving",
    std::bind(&DecisionMakerNode::updateDrivingState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::EXIT, "Driving",
    std::bind(&DecisionMakerNode::exitDrivingState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::ENTRY, "DrivingMissionChange",
    std::bind(&DecisionMakerNode::entryDrivingMissionChangeState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::UPDATE, "DrivingMissionChange",
    std::bind(&DecisionMakerNode::updateDrivingMissionChangeState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::UPDATE, "MissionChangeSucceeded",
    std::bind(&DecisionMakerNode::updateMissionChangeSucceededState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::UPDATE, "MissionChangeFailed",
    std::bind(&DecisionMakerNode::updateMissionChangeFailedState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::ENTRY, "MissionComplete",
    std::bind(&DecisionMakerNode::entryMissionCompleteState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::UPDATE, "MissionComplete",
    std::bind(&DecisionMakerNode::updateMissionCompleteState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::ENTRY, "MissionAborted",
    std::bind(&DecisionMakerNode::entryMissionAbortedState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
    state_machine::CallbackType::UPDATE, "MissionAborted",
    std::bind(&DecisionMakerNode::updateMissionAbortedState, this, std::placeholders::_1, 0));

  /*** state behavior ***/
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "Stopping",
    std::bind(&DecisionMakerNode::updateStoppingState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "BehaviorEmergency",
    std::bind(&DecisionMakerNode::updateBehaviorEmergencyState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::EXIT, "BehaviorEmergency",
    std::bind(&DecisionMakerNode::exitBehaviorEmergencyState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "Moving",
    std::bind(&DecisionMakerNode::updateMovingState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "FreeArea",
    std::bind(&DecisionMakerNode::updateFreeAreaState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "LaneArea",
    std::bind(&DecisionMakerNode::updateLaneAreaState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "Cruise",
    std::bind(&DecisionMakerNode::updateCruiseState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::ENTRY, "LeftTurn",
    std::bind(&DecisionMakerNode::entryTurnState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "LeftTurn",
    std::bind(&DecisionMakerNode::updateLeftTurnState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::ENTRY, "RightTurn",
    std::bind(&DecisionMakerNode::entryTurnState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "RightTurn",
    std::bind(&DecisionMakerNode::updateRightTurnState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::ENTRY, "Straight",
    std::bind(&DecisionMakerNode::entryTurnState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "Straight",
    std::bind(&DecisionMakerNode::updateStraightState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::ENTRY, "Back",
    std::bind(&DecisionMakerNode::entryTurnState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "Back",
    std::bind(&DecisionMakerNode::updateBackState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::ENTRY, "LeftLaneChange",
    std::bind(&DecisionMakerNode::entryLaneChangeState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "LeftLaneChange",
    std::bind(&DecisionMakerNode::updateLeftLaneChangeState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "CheckLeftLane",
    std::bind(&DecisionMakerNode::updateCheckLeftLaneState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "ChangeToLeft",
    std::bind(&DecisionMakerNode::updateChangeToLeftState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::ENTRY, "RightLaneChange",
    std::bind(&DecisionMakerNode::entryLaneChangeState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "RightLaneChange",
    std::bind(&DecisionMakerNode::updateRightLaneChangeState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "CheckRightLane",
    std::bind(&DecisionMakerNode::updateCheckRightLaneState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "ChangeToRight",
    std::bind(&DecisionMakerNode::updateChangeToRightState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "BusStop",
    std::bind(&DecisionMakerNode::updateBusStopState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "PullIn",
    std::bind(&DecisionMakerNode::updatePullInState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "PullOut",
    std::bind(&DecisionMakerNode::updatePullOutState, this, std::placeholders::_1, 0));
  ctx_behavior->setCallback(
    state_machine::CallbackType::UPDATE, "Parking",
    std::bind(&DecisionMakerNode::updateParkingState, this, std::placeholders::_1, 0));

  /*** state motion ***/
  ctx_motion->setCallback(
    state_machine::CallbackType::UPDATE, "WaitDriveReady",
    std::bind(&DecisionMakerNode::updateWaitDriveReadyState, this, std::placeholders::_1, 0));
  ctx_motion->setCallback(
    state_machine::CallbackType::UPDATE, "WaitEngage",
    std::bind(&DecisionMakerNode::updateWaitEngageState, this, std::placeholders::_1, 0));
  ctx_motion->setCallback(
    state_machine::CallbackType::UPDATE, "MotionEmergency",
    std::bind(&DecisionMakerNode::updateMotionEmergencyState, this, std::placeholders::_1, 0));
  ctx_motion->setCallback(
    state_machine::CallbackType::ENTRY, "Drive",
    std::bind(&DecisionMakerNode::entryDriveState, this, std::placeholders::_1, 0));
  ctx_motion->setCallback(
    state_machine::CallbackType::UPDATE, "Drive",
    std::bind(&DecisionMakerNode::updateDriveState, this, std::placeholders::_1, 0));
  ctx_motion->setCallback(
    state_machine::CallbackType::ENTRY, "Go",
    std::bind(&DecisionMakerNode::entryGoState, this, std::placeholders::_1, 0));
  ctx_motion->setCallback(
    state_machine::CallbackType::UPDATE, "Go",
    std::bind(&DecisionMakerNode::updateGoState, this, std::placeholders::_1, 0));
  ctx_motion->setCallback(
    state_machine::CallbackType::UPDATE, "Wait",
    std::bind(&DecisionMakerNode::updateWaitState, this, std::placeholders::_1, 0));
  ctx_motion->setCallback(
    state_machine::CallbackType::UPDATE, "Stop",
    std::bind(&DecisionMakerNode::updateStopState, this, std::placeholders::_1, 1));
  ctx_motion->setCallback(
    state_machine::CallbackType::UPDATE, "StopLine",
    std::bind(&DecisionMakerNode::updateStoplineState, this, std::placeholders::_1, 0));
  ctx_motion->setCallback(
    state_machine::CallbackType::UPDATE, "OrderedStop",
    std::bind(&DecisionMakerNode::updateOrderedStopState, this, std::placeholders::_1, 1));
  ctx_motion->setCallback(
    state_machine::CallbackType::EXIT, "OrderedStop",
    std::bind(&DecisionMakerNode::exitOrderedStopState, this, std::placeholders::_1, 1));
  ctx_motion->setCallback(
    state_machine::CallbackType::UPDATE, "ReservedStop",
    std::bind(&DecisionMakerNode::updateReservedStopState, this, std::placeholders::_1, 1));
  ctx_motion->setCallback(
    state_machine::CallbackType::EXIT, "ReservedStop",
    std::bind(&DecisionMakerNode::exitReservedStopState, this, std::placeholders::_1, 1));

  ctx_vehicle->nextState("started");
  ctx_mission->nextState("started");
  ctx_behavior->nextState("started");
  ctx_motion->nextState("started");
}

void DecisionMakerNode::createSubscriber(void)
{
  // Config subscriber
  Subs["config/decision_maker"] =
    nh_.subscribe("config/decision_maker", 3, &DecisionMakerNode::callbackFromConfig, this);
  Subs["state_cmd"] = nh_.subscribe("state_cmd", 1, &DecisionMakerNode::callbackFromStateCmd, this);
  Subs["vehicle_engage"] = nh_.subscribe("vehicle/engage", 1, &DecisionMakerNode::callbackFromEngage, this);
  Subs["vehicle_status"] = nh_.subscribe("vehicle_status", 1, &DecisionMakerNode::callbackFromStatus, this);
  Subs["detection/lidar_detector/objects"] =
    nh_.subscribe("detection/lidar_detector/objects", 1, &DecisionMakerNode::callbackFromDetection, this);
  Subs["current_velocity"] =
    nh_.subscribe("current_velocity", 1, &DecisionMakerNode::callbackFromCurrentVelocity, this);
  Subs["obstacle_waypoint"] =
    nh_.subscribe("obstacle_waypoint", 1, &DecisionMakerNode::callbackFromObstacleWaypoint, this);
  Subs["stopline_waypoint"] =
    nh_.subscribe("stopline_waypoint", 1, &DecisionMakerNode::callbackFromStoplineWaypoint, this);
  Subs["change_flag"] = nh_.subscribe("change_flag", 1, &DecisionMakerNode::callbackFromLaneChangeFlag, this);
  if (!ignore_map_ && use_lanelet_map_)
  {
    Subs["lanelet_map"] = nh_.subscribe("lanelet_map_bin", 1, &DecisionMakerNode::callbackFromLanelet2Map, this);
  }
}
void DecisionMakerNode::createPublisher(void)
{
  // pub
  Pubs["state/stopline_wpidx"] = nh_.advertise<std_msgs::Int32>("state/stopline_wpidx", 1, false);

  // for controlling other planner
  Pubs["lane_waypoints_array"] = nh_.advertise<autoware_msgs::LaneArray>(TPNAME_CONTROL_LANE_WAYPOINTS_ARRAY, 10, true);
  Pubs["light_color"] = nh_.advertise<autoware_msgs::TrafficLight>("light_color_managed", 1);

  // for controlling vehicle
  Pubs["lamp_cmd"] = nh_.advertise<autoware_msgs::LampCmd>("lamp_cmd", 1);

  // for visualize status
  Pubs["state"] = private_nh_.advertise<std_msgs::String>("state", 1, true);
  Pubs["state_msg"] = private_nh_.advertise<autoware_msgs::State>("state_msg", 1, true);
  Pubs["state_overlay"] = private_nh_.advertise<jsk_rviz_plugins::OverlayText>("state_overlay", 1);
  Pubs["available_transition"] = private_nh_.advertise<std_msgs::String>("available_transition", 1, true);
  Pubs["stop_cmd_location"] = private_nh_.advertise<autoware_msgs::VehicleLocation>("stop_location", 1, true);
  Pubs["stop_zone"] = private_nh_.advertise<visualization_msgs::Marker>("stop_zone_visualizer", 1);

  // for debug
  Pubs["target_velocity_array"] = nh_.advertise<std_msgs::Float64MultiArray>("target_velocity_array", 1);
  Pubs["operator_help_text"] = private_nh_.advertise<jsk_rviz_plugins::OverlayText>("operator_help_text", 1, true);
}

void DecisionMakerNode::initROS()
{
  // for subscribe callback function

  createSubscriber();
  createPublisher();

  if (ignore_map_)
  {
    ROS_WARN("Ignoring map data");
  }
  else
  {
    if (use_lanelet_map_)
    {
      initLaneletMap();
    }
    else
    {
      initVectorMap();
    }
  }

  spinners = std::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(3));
  spinners->start();

  update_msgs();
}

void DecisionMakerNode::initLaneletMap(void)
{
  int _index = 0;
  bool ll2_map_loaded = false;
  while (!ll2_map_loaded && ros::ok())
  {
    ros::spinOnce();
    ROS_INFO_THROTTLE(2, "Waiting for lanelet map topic");
    ll2_map_loaded = isEventFlagTrue("lanelet2_map_loaded");
    ros::Duration(0.1).sleep();
  }

  if (ll2_map_loaded)
  {
    lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_);
    for (const auto& lanelet : all_lanelets)
    {
      geometry_msgs::Point _prev_point;
      CrossRoadArea intersect;
      intersect.id = _index++;

      double x_avg = 0.0, x_min = 0.0, x_max = 0.0;
      double y_avg = 0.0, y_min = 0.0, y_max = 0.0;
      double z_avg = 0.0;
      int points_count = 0;

      // skip lanelets that do not belong to an intersection
      if (lanelet.attributeOr("turn_direction", "empty") == "empty")
      {
        continue;
      }

      std::vector<geometry_msgs::Point> _points;
      // for every point that makes up a line (rightBound)
      for (const auto& rb_pt : lanelet.rightBound())
      {
        geometry_msgs::Point _point;
        _point.x = rb_pt.x();
        _point.y = rb_pt.y();
        _point.z = rb_pt.z();

        // skip repeated points
        if (_prev_point.x == _point.x && _prev_point.y == _point.y)
        {
          continue;
        }

        _prev_point = _point;
        points_count++;
        _points.push_back(_point);

        // calc a centroid point and about intersects_ size
        x_avg += _point.x;
        y_avg += _point.y;
        z_avg += _point.z;
        x_min = (x_min == 0.0) ? _point.x : std::min(_point.x, x_min);
        x_max = (x_max == 0.0) ? _point.x : std::max(_point.x, x_max);
        y_min = (y_min == 0.0) ? _point.y : std::min(_point.y, y_min);
        y_max = (y_max == 0.0) ? _point.y : std::max(_point.y, y_max);
      }  // points iter

      // for every point that makes up a line (leftBound)
      for (const auto& lb_pt : lanelet.leftBound())
      {
        geometry_msgs::Point _point;
        _point.x = lb_pt.x();
        _point.y = lb_pt.y();
        _point.z = lb_pt.z();

        // skip repeated points
        if (_prev_point.x == _point.x && _prev_point.y == _point.y)
        {
          continue;
        }

        _prev_point = _point;
        points_count++;
        _points.push_back(_point);

        // calc a centroid point and about intersects_ size
        x_avg += _point.x;
        y_avg += _point.y;
        z_avg += _point.z;
        x_min = (x_min == 0.0) ? _point.x : std::min(_point.x, x_min);
        x_max = (x_max == 0.0) ? _point.x : std::max(_point.x, x_max);
        y_min = (y_min == 0.0) ? _point.y : std::min(_point.y, y_min);
        y_max = (y_max == 0.0) ? _point.y : std::max(_point.y, y_max);
      }  // points iter

      // accumulate all points from both rightBound and leftBound
      for (const auto& pt : _points)
      {
        intersect.points.push_back(pt);
      }

      // centroid point of intersection bbox
      intersect.bbox.pose.position.x = x_avg / static_cast<double>(points_count);
      intersect.bbox.pose.position.y = y_avg / static_cast<double>(points_count);
      intersect.bbox.pose.position.z = z_avg / static_cast<double>(points_count);
      // dimensions of intersection bbox
      intersect.bbox.dimensions.x = x_max - x_min;
      intersect.bbox.dimensions.y = y_max - y_min;
      intersect.bbox.dimensions.z = 2;
      intersect.bbox.label = 1;

      // combine lanelets that belong to a same intersection
      bool first_instance = true;
      for (auto& intersect_itr : intersects_)
      {
        double _dist = amathutils::find_distance(intersect.bbox.pose.position, intersect_itr.bbox.pose.position);
        // if distance between one intersection and another is less than 20.0 m, consider it the same intersection
        if (_dist < 20.0)
        {
          intersect_itr.bbox.pose.position.x =
            (intersect_itr.bbox.pose.position.x + intersect.bbox.pose.position.x) / 2.0;
          intersect_itr.bbox.pose.position.y =
            (intersect_itr.bbox.pose.position.y + intersect.bbox.pose.position.y) / 2.0;
          intersect_itr.bbox.pose.position.z =
            (intersect_itr.bbox.pose.position.z + intersect.bbox.pose.position.z) / 2.0;
          // TODO(Steven): fix xy_min and xy_max as they're not calculated correctly but not currently used

          first_instance = false;
          break;
        }
      }

      if (first_instance)
      {
        intersects_.push_back(intersect);
      }
    }

    // stop zone
    lanelet::ConstLineStrings3d stoplines = lanelet::utils::query::getStopSignStopLines(all_lanelets);

    // find all points where lanes intersect with stoplines
    for (const auto& stopline : stoplines)
    {
      // skip invalid stopline (line without point)
      if (stopline.empty())
      {
        continue;
      }

      geometry_msgs::Point stop_bp = lanelet::utils::conversion::toGeomMsgPt(stopline.front());
      geometry_msgs::Point stop_fp = lanelet::utils::conversion::toGeomMsgPt(stopline.back());

      geometry_msgs::Point stop_zone;
      stop_zone.x = (stop_bp.x + stop_fp.x) / 2.0;
      stop_zone.y = (stop_bp.y + stop_fp.y) / 2.0;
      stop_zone.z = (stop_bp.z + stop_fp.z) / 2.0;
      ROS_DEBUG("[stop_zone] (%f , %f)\n", stop_zone.x, stop_zone.y);

      double dist = DBL_MAX;
      int intersection_id = -1;
      // find intersection which stopline belongs to by finding the smallest distance
      // between the stopline and intersection center point
      for (const auto& intersect : intersects_)
      {
        double d = amathutils::distanceFromSegment(stop_bp, stop_fp, intersect.bbox.pose.position);
        if (dist > d)
        {
          dist = d;
          intersection_id = intersect.id;
        }
      }
      if (intersection_id != -1)
      {
        for (auto& intersect : intersects_)
        {
          if (intersect.id == intersection_id)
          {
            CrossRoadArea::StopArea stop_area;
            stop_area.stopline_id = stopline.id();
            stop_area.stop_point = stop_zone;
            stop_area.is_safe = false;
            intersect.stops.push_back(stop_area);
            ROS_DEBUG("intersection_id: %d | stopline.id: %d | stop_point: (%f , %f)\n",
                           intersection_id, static_cast<int>(stopline.id()), stop_zone.x, stop_zone.y);
          }
        }
      }
    }

    displayStopZoneInit();
  }
}

void DecisionMakerNode::initVectorMap(void)
{
  int _index = 0;
  bool vmap_loaded = false;

  while (!vmap_loaded && ros::ok())
  {
    // Map must be populated before setupStateCallback() is called
    // in DecisionMakerNode constructor
    ROS_INFO("Subscribing to vector map topics.");

    g_vmap.subscribe(nh_, Category::POINT | Category::LINE | Category::VECTOR | Category::AREA |
      Category::STOP_LINE | Category::ROAD_SIGN | Category::CROSS_ROAD | Category::WHITE_LINE | Category::LANE ,
      ros::Duration(1.0));

    vmap_loaded =
        g_vmap.hasSubscribed(Category::POINT | Category::LINE | Category::LANE | Category::AREA |
                              Category::STOP_LINE | Category::ROAD_SIGN);

    if (!vmap_loaded)
    {
      ROS_WARN_THROTTLE(5, "Necessary vectormap topics have not been published.");
      ROS_WARN_THROTTLE(5, "DecisionMaker will wait until the vectormap has been loaded.");
    }
    else
    {
      ROS_INFO("Vectormap loaded.");
    }
  }

  const std::vector<CrossRoad> crossroads =
    g_vmap.findByFilter([](const CrossRoad& crossroad) { return true; });  // NOLINT

  if (crossroads.empty())
  {
    ROS_INFO("crossroads have not found\n");
    return;
  }

  // for every cross_road (intersection)
  for (const auto& cross_road : crossroads)
  {
    geometry_msgs::Point _prev_point;
    Area area = g_vmap.findByKey(Key<Area>(cross_road.aid));
    CrossRoadArea intersect;
    intersect.id = _index++;
    intersect.area_id = area.aid;

    double x_avg = 0.0, x_min = 0.0, x_max = 0.0;
    double y_avg = 0.0, y_min = 0.0, y_max = 0.0;
    double z = 0.0;
    int points_count = 0;

    const std::vector<Line> lines =
      g_vmap.findByFilter(
        [&area](const Line& line)
        {
          return area.slid <= line.lid && line.lid <= area.elid;
        }
      );  // NOLINT

    if ( lines.size() < 3 )
    {
      continue;
    }

    // for every line that makes up an intersection
    for (const auto& line : lines)
    {
      const std::vector<Point> points =
        g_vmap.findByFilter([&line](const Point& point) { return line.bpid == point.pid; });  // NOLINT

      // for every point that makes up a line
      for (const auto& point : points)
      {
        geometry_msgs::Point _point;
        _point.x = point.ly;
        _point.y = point.bx;
        _point.z = point.h;

        // skip repeated points
        if (_prev_point.x == _point.x && _prev_point.y == _point.y)
        {
          continue;
        }

        _prev_point = _point;
        points_count++;
        intersect.points.push_back(_point);

        // calc a centroid point and about intersects_ size
        x_avg += _point.x;
        y_avg += _point.y;
        x_min = (x_min == 0.0) ? _point.x : std::min(_point.x, x_min);
        x_max = (x_max == 0.0) ? _point.x : std::max(_point.x, x_max);
        y_min = (y_min == 0.0) ? _point.y : std::min(_point.y, y_min);
        y_max = (y_max == 0.0) ? _point.y : std::max(_point.y, y_max);
        z = _point.z;
      }  // points iter
    }  // line iter

    // centroid point of intersection bbox
    intersect.bbox.pose.position.x = x_avg / static_cast<double>(points_count);
    intersect.bbox.pose.position.y = y_avg / static_cast<double>(points_count);
    intersect.bbox.pose.position.z = z;
    // dimensions of intersection bbox
    intersect.bbox.dimensions.x = x_max - x_min;
    intersect.bbox.dimensions.y = y_max - y_min;
    intersect.bbox.dimensions.z = 2;
    intersect.bbox.label = 1;
    intersects_.push_back(intersect);
  }

  // stop zone
  std::vector<StopLine> stoplines = g_vmap.findByFilter(
    [&](const StopLine& stopline)
    {
      return ((g_vmap.findByKey(Key<RoadSign>(stopline.signid)).type &
        (autoware_msgs::WaypointState::TYPE_STOP | autoware_msgs::WaypointState::TYPE_STOPLINE)) != 0);
    }
  );  // NOLINT
  const std::vector<Lane> lanes = g_vmap.findByFilter(
    [](const Lane& lane) { return true; });  // NOLINT

  // find all points where lanes intersect with stoplines
  for (const auto& stopline : stoplines)
  {
    if (stopline.linkid == 0)
    {
      continue;
    }

    // store points that make each stop line
    geometry_msgs::Point stop_bp =
        VMPoint2GeoPoint(g_vmap.findByKey(Key<Point>(g_vmap.findByKey(Key<Line>(stopline.lid)).bpid)));
    geometry_msgs::Point stop_fp =
        VMPoint2GeoPoint(g_vmap.findByKey(Key<Point>(g_vmap.findByKey(Key<Line>(stopline.lid)).fpid)));
    geometry_msgs::Point stop_zone;

    // find lane that intersects_ through the stopline
    for (const auto& lane : lanes)
    {
      geometry_msgs::Point lane_bp = VMPoint2GeoPoint(g_vmap.findByKey(Key<Point>(lane.bnid)));
      geometry_msgs::Point lane_fp = VMPoint2GeoPoint(g_vmap.findByKey(Key<Point>(lane.fnid)));

      if (amathutils::isIntersectLine(lane_bp, lane_fp, stop_bp, stop_fp))
      {
        geometry_msgs::Point int_point;
        amathutils::getIntersect(lane_bp, lane_fp, stop_bp, stop_fp, &int_point);
        stop_zone = int_point;
        break;
      }
    }

    double dist = DBL_MAX;
    int intersection_id = -1;
    // find intersection which stopline belongs to by finding the smallest distance
    // between the stopline and intersection center point
    for (const auto& intersect : intersects_)
    {
      double d = amathutils::distanceFromSegment(stop_bp, stop_fp, intersect.bbox.pose.position);
      if (dist > d)
      {
        dist = d;
        intersection_id = intersect.id;
      }
    }
    if (intersection_id != -1)
    {
      for (auto& intersect : intersects_)
      {
        if (intersect.id == intersection_id)
        {
          CrossRoadArea::StopArea stop_area;
          stop_area.stopline_id = stopline.id;
          stop_area.stop_point = stop_zone;
          stop_area.is_safe = false;
          intersect.stops.push_back(stop_area);
          ROS_DEBUG("intersection_id: %d | stopline.id: %d | stopline.linkid: %d | stop_point: (%f , %f)\n",
                           intersection_id, stopline.id, stopline.linkid, stop_zone.x, stop_zone.y);
        }
      }
    }
  }

  displayStopZoneInit();
}

void DecisionMakerNode::displayStopZoneInit()
{
  // stop_zone_marker_ to visualize clearance at each intersection's stop zones
  stop_zone_marker_.header.frame_id = "/map";
  stop_zone_marker_.header.stamp = ros::Time();
  stop_zone_marker_.ns = "stop_zone";
  stop_zone_marker_.id = 0;
  stop_zone_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  stop_zone_marker_.action = visualization_msgs::Marker::ADD;

  // set scale and color
  const double scale = 3.7;
  stop_zone_marker_.scale.x = scale;
  stop_zone_marker_.scale.y = scale;
  stop_zone_marker_.scale.z = scale;
  stop_zone_marker_.color.a = 0.5;
  stop_zone_marker_.color.r = 1.0;
  stop_zone_marker_.color.g = 0.0;
  stop_zone_marker_.color.b = 0.0;
  stop_zone_marker_.frame_locked = true;
}
}  // namespace decision_maker
