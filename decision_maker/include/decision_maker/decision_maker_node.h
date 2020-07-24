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

#ifndef DECISION_MAKER_DECISION_MAKER_NODE_H
#define DECISION_MAKER_DECISION_MAKER_NODE_H

#include "decision_maker/cross_road_area.h"

#include <cstdio>
#include <map>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <ros/spinner.h>

#include <autoware_config_msgs/ConfigDecisionMaker.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/State.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleLocation.h>
#include <autoware_msgs/Waypoint.h>
#include <autoware_msgs/WaypointState.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

#include <amathutils_lib/amathutils.hpp>
#include <state_machine_lib/state_context.hpp>
#include <tf/transform_listener.h>
#include <vector_map/vector_map.h>

#include <lanelet2_io/Io.h>
#include <lanelet2_routing/Route.h>

namespace decision_maker
{
using vector_map::Area;
using vector_map::Category;
using vector_map::CrossRoad;
using vector_map::Key;
using vector_map::Line;
using vector_map::Point;
using vector_map::RoadSign;
using vector_map::StopLine;
using vector_map::VectorMap;
using vector_map::WhiteLine;

using cstring_t = const std::string;

constexpr double ANGLE_NEUTRAL = 0;
constexpr double ANGLE_CURVE = 40;
constexpr double ANGLE_LEFT = (ANGLE_NEUTRAL - ANGLE_CURVE);
constexpr double ANGLE_RIGHT = (ANGLE_NEUTRAL + ANGLE_CURVE);

constexpr char TPNAME_BASED_LANE_WAYPOINTS_ARRAY[] = "/based/lane_waypoints_array";
constexpr char TPNAME_CONTROL_LANE_WAYPOINTS_ARRAY[] = "/lane_waypoints_array";
constexpr int LAMP_ON = 1;
constexpr int LAMP_OFF = 0;

enum class E_Lamp : int32_t
{
  LAMP_EMPTY = -1,
  LAMP_CLEAR = 0,
  LAMP_RIGHT = 1,
  LAMP_LEFT = 2,
  LAMP_HAZARD = 3
};
enum class E_ChangeFlags : int32_t
{
  STRAIGHT,
  RIGHT,
  LEFT,

  UNKNOWN = -1,
};

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t)
{
  return static_cast<typename std::underlying_type<T>::type>(t);
}

struct AutowareStatus
{
  std::map<std::string, bool> EventFlags;

  // planning status
  autoware_msgs::LaneArray using_lane_array;  // with wpstate
  autoware_msgs::LaneArray based_lane_array;
  autoware_msgs::Lane finalwaypoints;
  int closest_waypoint;
  int obstacle_waypoint;
  int stopline_waypoint;
  int change_flag;

  // vehicle status
  geometry_msgs::Pose pose;
  double velocity;  // kmph

  int found_stopsign_idx;
  int prev_stopped_wpidx;
  int ordered_stop_idx;
  int prev_ordered_idx;

  AutowareStatus(void) :
    closest_waypoint(-1),
    obstacle_waypoint(-1),
    stopline_waypoint(-1),
    velocity(0),
    found_stopsign_idx(-1),
    prev_stopped_wpidx(-1),
    ordered_stop_idx(-1),
    prev_ordered_idx(-1)
  {
  }
};

class DecisionMakerNode
{
  friend class TestClass;
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Publishers
  std::unordered_map<std::string, ros::Publisher> Pubs;
  // Subscribers
  std::unordered_map<std::string, ros::Subscriber> Subs;

  std::shared_ptr<ros::AsyncSpinner> spinners;

  AutowareStatus current_status_;

  std::vector<CrossRoadArea> intersects;

  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::routing::RoutingGraphPtr routing_graph_;

  // Param
  bool auto_mission_reload_;
  bool auto_engage_;
  bool auto_mission_change_;
  bool use_fms_;
  bool ignore_map_;
  bool insert_stop_line_wp_;
  int param_num_of_steer_behind_;
  int distance_before_lane_change_signal_;
  double change_threshold_dist_;
  double change_threshold_angle_;
  double goal_threshold_dist_;
  double goal_threshold_vel_;
  double stopped_vel_;
  int stopline_reset_count_;
  bool sim_mode_;
  bool use_lanelet_map_;
  std::string stop_sign_id_;

  // initialization method
  void initROS();
  void initVectorMap(void);
  void initLaneletMap(void);

  void createSubscriber(void);
  void createPublisher(void);

  // looping method
  void update(void);
  void update_msgs(void);

  void publishOperatorHelpMessage(const cstring_t& message);
  void publishLampCmd(const E_Lamp& status);
  void publishStoplineWaypointIdx(const int wp_idx);

  /* decision */
  void tryNextState(cstring_t& key);
  bool isArrivedGoal(void) const;
  bool isLocalizationConvergence(const geometry_msgs::Point& _current_point) const;
  void insertPointWithinCrossRoad(const std::vector<CrossRoadArea>& _intersects, autoware_msgs::LaneArray& lane_array);
  void setWaypointStateUsingVectorMap(autoware_msgs::LaneArray& lane_array);
  void setWaypointStateUsingLanelet2Map(autoware_msgs::LaneArray& lane_array);
  bool drivingMissionCheck(void);

  double calcIntersectWayAngle(const autoware_msgs::Lane& laneinArea);
  double getDistToWaypointIdx(const int wpidx) const;
  double calcRequiredDistForStop(void) const;

  uint8_t getSteeringStateFromWaypoint(void);
  uint8_t getEventStateFromWaypoint(void);
  std::pair<uint8_t, int> getStopSignStateFromWaypoint(void);

  // current decision maker is support only lane area
  bool isVehicleOnLaneArea(void)
  {
    return true;
  }

  void setupStateCallback(void);

  /*
   * state callback
   **/

  /*** state vehicle ***/
  // entry callback
  void entryInitState(cstring_t& state_name, int status);
  void entrySensorInitState(cstring_t& state_name, int status);
  void entryLocalizationInitState(cstring_t& state_name, int status);
  void entryPlanningInitState(cstring_t& state_name, int status);
  void entryVehicleInitState(cstring_t& state_name, int status);
  void entryVehicleReadyState(cstring_t& state_name, int status);
  void entryVehicleEmergencyState(cstring_t& state_name, int status);
  // update callback
  void updateInitState(cstring_t& state_name, int status);
  void updateSensorInitState(cstring_t& state_name, int status);
  void updateLocalizationInitState(cstring_t& state_name, int status);
  void updatePlanningInitState(cstring_t& state_name, int status);
  void updateVehicleInitState(cstring_t& state_name, int status);
  void updateVehicleReadyState(cstring_t& state_name, int status);
  void updateBatteryChargingState(cstring_t& state_name, int status);
  void updateVehicleEmergencyState(cstring_t& state_name, int status);
  // exit callback

  /*** state mission ***/
  // entry callback
  void entryMissionInitState(cstring_t& state_name, int status);
  void entryWaitOrderState(cstring_t& state_name, int status);
  void entryMissionCheckState(cstring_t& state_name, int status);
  void entryDriveReadyState(cstring_t& state_name, int status);
  void entryDrivingState(cstring_t& state_name, int status);
  void entryDrivingMissionChangeState(cstring_t& state_name, int status);
  void entryMissionAbortedState(cstring_t& state_name, int status);
  void entryMissionCompleteState(cstring_t& state_name, int status);
  // update callback
  void updateMissionInitState(cstring_t& state_name, int status);
  void updateWaitOrderState(cstring_t& state_name, int status);
  void updateMissionCheckState(cstring_t& state_name, int status);
  void updateDriveReadyState(cstring_t& state_name, int status);
  void updateDrivingState(cstring_t& state_name, int status);
  void updateDrivingMissionChangeState(cstring_t& state_name, int status);
  void updateMissionChangeSucceededState(cstring_t& state_name, int status);
  void updateMissionChangeFailedState(cstring_t& state_name, int status);
  void updateMissionCompleteState(cstring_t& state_name, int status);
  void updateMissionAbortedState(cstring_t& state_name, int status);
  // exit callback
  void exitWaitOrderState(cstring_t& state_name, int status);
  void exitDrivingState(cstring_t& state_name, int status);
  // void exitWaitMissionOrderState(cstring_t& state_name, int status);

  /*** state behavior ***/
  // entry callback
  void entryTurnState(cstring_t& state_name, int status);
  void entryLaneChangeState(cstring_t& state_name, int status);
  // update callback
  void updateStoppingState(cstring_t& state_name, int status);
  void updateBehaviorEmergencyState(cstring_t& state_name, int status);
  void updateMovingState(cstring_t& state_name, int status);
  void updateLaneAreaState(cstring_t& state_name, int status);
  void updateFreeAreaState(cstring_t& state_name, int status);
  void updateCruiseState(cstring_t& state_name, int status);
  void updateBusStopState(cstring_t& state_name, int status);
  void updateParkingState(cstring_t& state_name, int status);
  void updateLeftTurnState(cstring_t& state_name, int status);
  void updateRightTurnState(cstring_t& state_name, int status);
  void updateStraightState(cstring_t& state_name, int status);
  void updateBackState(cstring_t& state_name, int status);
  void updateLeftLaneChangeState(cstring_t& state_name, int status);
  void updateRightLaneChangeState(cstring_t& state_name, int status);
  void updatePullInState(cstring_t& state_name, int status);
  void updatePullOutState(cstring_t& state_name, int status);
  void updateCheckLeftLaneState(cstring_t& state_name, int status);
  void updateCheckRightLaneState(cstring_t& state_name, int status);
  void updateChangeToLeftState(cstring_t& state_name, int status);
  void updateChangeToRightState(cstring_t& state_name, int status);
  // exit callback
  void exitBehaviorEmergencyState(cstring_t& state_name, int status);

  /*** state motion ***/
  // entry callback
  void entryDriveState(cstring_t& state_name, int status);
  void entryGoState(cstring_t& state_name, int status);
  // update callback
  void updateWaitDriveReadyState(cstring_t& state_name, int status);
  void updateWaitEngageState(cstring_t& state_name, int status);
  void updateDriveState(cstring_t& state_name, int status);
  void updateMotionEmergencyState(cstring_t& state_name, int status);
  void updateGoState(cstring_t& state_name, int status);
  void updateWaitState(cstring_t& state_name, int status);
  void updateStopState(cstring_t& state_name, int status);
  void updateStoplineState(cstring_t& state_name, int status);
  void updateOrderedStopState(cstring_t& state_name, int status);
  void updateReservedStopState(cstring_t& state_name, int status);
  // exit callback
  void exitWaitState(cstring_t& state_name, int status);
  void exitStopState(cstring_t& state_name, int status);
  void exitOrderedStopState(cstring_t& state_name, int status);
  void exitReservedStopState(cstring_t& state_name, int status);

  // callback by topic subscribing
  void callbackFromFilteredPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStamped& msg);
  void callbackFromCurrentPose(const geometry_msgs::PoseStamped& msg);
  void callbackFromClosestWaypoint(const std_msgs::Int32& msg);
  void callbackFromLightColor(const ros::MessageEvent<autoware_msgs::TrafficLight const>& event);
  void callbackFromLaneChangeFlag(const std_msgs::Int32& msg);
  void callbackFromFinalWaypoint(const autoware_msgs::Lane& msg);
  void callbackFromLaneWaypoint(const autoware_msgs::LaneArray& msg);
  void callbackFromSimPose(const geometry_msgs::PoseStamped& msg);
  void callbackFromConfig(const autoware_config_msgs::ConfigDecisionMaker& msg);
  void callbackFromStateCmd(const std_msgs::String& msg);
  void callbackFromEngage(const std_msgs::Bool& msg);
  void callbackFromObstacleWaypoint(const std_msgs::Int32& msg);
  void callbackFromStoplineWaypoint(const std_msgs::Int32& msg);
  void callbackFromStopOrder(const std_msgs::Int32& msg);
  void callbackFromClearOrder(const std_msgs::Int32& msg);
  void callbackFromLanelet2Map(const autoware_lanelet2_msgs::MapBin::ConstPtr& msg);

  void setEventFlag(cstring_t& key, const bool& value)
  {
    current_status_.EventFlags[key] = value;
  }

  bool isEventFlagTrue(std::string key)
  {
    if (current_status_.EventFlags.count(key) == 0)
    {
      current_status_.EventFlags[key] = false;
    }
    return current_status_.EventFlags[key];
  }

public:
  std::unique_ptr<state_machine::StateContext> ctx_vehicle;
  std::unique_ptr<state_machine::StateContext> ctx_mission;
  std::unique_ptr<state_machine::StateContext> ctx_behavior;
  std::unique_ptr<state_machine::StateContext> ctx_motion;
  VectorMap g_vmap;

  DecisionMakerNode();

  void run();

  bool isSubscriberRegistered(cstring_t& topic_name)
  {
    return Subs.count(topic_name) ? true : false;
  }

  static geometry_msgs::Point VMPoint2GeoPoint(const vector_map_msgs::Point& vp)
  {
    geometry_msgs::Point gp;
    gp.x = vp.ly;
    gp.y = vp.bx;
    gp.z = vp.h;
    return gp;
  }
};

}  // namespace decision_maker

#endif  // DECISION_MAKER_DECISION_MAKER_NODE_H
