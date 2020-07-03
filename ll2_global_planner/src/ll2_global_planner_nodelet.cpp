/*
* Unpublished Copyright (c) 2009-2019 AutonomouStuff, All Rights Reserved.
*
* This file is part of the ll2_global_planner which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#include "ll2_global_planner/ll2_global_planner_nodelet.hpp"

#include <string>
#include <vector>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <autoware_msgs/LaneArray.h>

#include <lanelet2_extension/utility/message_conversion.h>
// #include <lanelet2_projection/UTM.h>
#include <lanelet2_extension/projection/mgrs_projector.h>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

using namespace lanelet;

namespace ll2_global_planner
{

Ll2GlobalPlannerNl::Ll2GlobalPlannerNl() :
  tf_listener(this->tf_buffer)
{}

void Ll2GlobalPlannerNl::onInit()
{
  this->nh = getNodeHandle();
  this->pnh = getPrivateNodeHandle();
  this->loadParams();

  // Publishers
  this->waypoints_pub = nh.advertise<autoware_msgs::LaneArray>("based/lane_waypoints_raw", 1, true);

  // Subscribers
  this->lanelet_sub = nh.subscribe("lanelet_map_bin", 1, &Ll2GlobalPlannerNl::laneletMapCb, this);
  this->posegoal_sub = nh.subscribe("move_base_simple/goal", 1, &Ll2GlobalPlannerNl::poseGoalCb, this);
}

void Ll2GlobalPlannerNl::loadParams()
{
    ROS_INFO("Parameters Loaded");
}

void Ll2GlobalPlannerNl::laneletMapCb(const autoware_lanelet2_msgs::MapBin& map_msg)
{
  this->lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(map_msg, this->lanelet_map);
  this->initialized = true;
  ROS_INFO("Loaded Lanelet map");
}

void Ll2GlobalPlannerNl::poseGoalCb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  if (!this->initialized)
  {
    ROS_WARN("Waiting for lanelet2 map");
    return;
  }

  // GPS POINT CONVERSTION
  // GPSPoint gps_point;
  // gps_point.lat = 45.320147;
  // gps_point.lon = -75.760375;
  // projection::MGRSProjector projector;
  // BasicPoint3d local_point_3d = projector.forward(gps_point);
  // BasicPoint2d local_point(local_point_3d.x(), local_point_3d.y());


  // Find the nearest lanelet to the goal point
  BasicPoint2d local_point(pose_msg->pose.position.x, pose_msg->pose.position.y);
  geometry_msgs::Point goal_point(pose_msg->pose.position);

  Lanelet goal_lanelet = getNearestLanelet(local_point);

  // ROS_WARN("LANE ID: %d", nearest_lanelet.id());

  // Get the current vehicle position
  geometry_msgs::TransformStamped tf_msg;
  try
  {
    tf_msg = this->tf_buffer.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_THROTTLE(2, "%s", ex.what());
    ROS_WARN_THROTTLE(2, "Waiting for map -> base_link transform");
    return;
  }

  BasicPoint2d starting_point(
    tf_msg.transform.translation.x, tf_msg.transform.translation.y);
  Lanelet starting_lanelet = getNearestLanelet(starting_point);

  // Plan a route from current vehicle position
  traffic_rules::TrafficRulesPtr traffic_rules{traffic_rules::TrafficRulesFactory::instance().create(Locations::Germany, Participants::Vehicle)};
  routing::RoutingGraphUPtr graph = routing::RoutingGraph::build(*this->lanelet_map, *traffic_rules);
  Optional<routing::LaneletPath> shortest_path_opt = graph->shortestPath(starting_lanelet, goal_lanelet);
  routing::LaneletPath shortest_path;

  if (!shortest_path_opt)
  {
    ROS_WARN("Could not find path in lanelet map!");
    return;
  }

  shortest_path = shortest_path_opt.value();
  LaneletSequence continuous_lane = shortest_path.getRemainingLane(shortest_path.begin());

  if (continuous_lane.size() != shortest_path.size())
  {
    ROS_WARN("This route contains a lane change which is currently unsupported");
    return;
  }

  ROS_INFO("Found a path containing %d lanelets", shortest_path.size());

  // Convert to autoware waypoints
  std::vector<autoware_msgs::Waypoint> waypoints;
  BasicPoint3d prev_point;
  int wp_id = 0;
  bool first_point = true;
  float smallest_goal_dist = std::numeric_limits<float>::infinity();
  int smallest_goal_wp_id = 0;

  // Loop over each lanelet
  for (auto lanelet : continuous_lane.lanelets())
  {
    std::string turn_direction = lanelet.attributeOr("turn_direction", "straight");
    traffic_rules::SpeedLimitInformation speed_limit = traffic_rules->speedLimit(lanelet);
    ConstLineString3d centerline = lanelet.centerline();
    int wp_length = centerline.size() - 1;

    // Loop over each centerline point
    for (int i = 0; i <= wp_length; i++)
    {
      auto point = centerline[i];

      // Check for duplicate points
      if (!first_point)
      {
        float dist = std::hypot(std::hypot(point.x() - prev_point.x(), point.y() - prev_point.y()), point.z() - prev_point.z());
        if (dist < 0.001)
        {
          continue;
        }
      }
      else
      {
        first_point = false;
      }

      autoware_msgs::Waypoint new_wp;
      new_wp.pose.pose.position.x = point.x();
      new_wp.pose.pose.position.y = point.y();
      new_wp.pose.pose.position.z = point.z();
      new_wp.twist.twist.linear.x = speed_limit.speedLimit.value(); // m/s

      // Set orientation of the point, aiming at next point
      if (i == wp_length)
      {
        new_wp.pose.pose.orientation = waypoints.back().pose.pose.orientation;
      }
      else
      {
        auto next_point = centerline[i+1];
        double yaw = atan2(next_point.y() - point.y(), next_point.x() - point.x());
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, yaw);
        tf2::convert(orientation, new_wp.pose.pose.orientation);
      }

      int steering_state = autoware_msgs::WaypointState::STR_STRAIGHT;
      if (turn_direction.compare("right") == 0)
      {
        steering_state = autoware_msgs::WaypointState::STR_RIGHT;
      }
      if (turn_direction.compare("left") == 0)
      {
        steering_state = autoware_msgs::WaypointState::STR_LEFT;
      }

      new_wp.wpstate.steering_state = steering_state;
      new_wp.gid = new_wp.lid = wp_id;

      // Calculate distance to goal
      float goal_dist = std::hypot(std::hypot(goal_point.x - point.x(), goal_point.y - point.y()), goal_point.z - point.z());
      if (goal_dist < smallest_goal_dist)
      {
        smallest_goal_dist = goal_dist;
        smallest_goal_wp_id = wp_id;
      }

      waypoints.push_back(new_wp);
      wp_id++;
      prev_point = point;
    }
  }

  autoware_msgs::Lane lane_msg;
  lane_msg.waypoints = std::vector<autoware_msgs::Waypoint>(waypoints.begin(), waypoints.begin() + smallest_goal_wp_id);

  lane_msg.header.stamp = ros::Time(0);
  lane_msg.header.frame_id = "map";
  lane_msg.is_blocked = false;

  autoware_msgs::LaneArray lane_array_msg;
  lane_array_msg.lanes.push_back(lane_msg);
  this->waypoints_pub.publish(lane_array_msg);
}

Lanelet Ll2GlobalPlannerNl::getNearestLanelet(lanelet::BasicPoint2d point)
{
  std::vector<std::pair<double, Lanelet>> closeLanelets = geometry::findNearest(this->lanelet_map->laneletLayer, point, 1);
  Lanelet nearest_lanelet = closeLanelets[0].second;

  return nearest_lanelet;
}

}  // namespace ll2_global_planner

PLUGINLIB_EXPORT_CLASS(ll2_global_planner::Ll2GlobalPlannerNl, nodelet::Nodelet);
