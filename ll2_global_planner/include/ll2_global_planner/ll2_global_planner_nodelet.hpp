/*
* Unpublished Copyright (c) 2009-2020 AutonomouStuff, All Rights Reserved.
*
* This file is part of the ll2_global_planner which is released under the MIT license.
* See file LICENSE included with this software or go to https://opensource.org/licenses/MIT for full license details.
*/

#ifndef ll2_global_planner_Ll2GlobalPlannerNl_H
#define ll2_global_planner_Ll2GlobalPlannerNl_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <lanelet2_core/LaneletMap.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <geometry_msgs/PoseStamped.h>


namespace ll2_global_planner {

class Ll2GlobalPlannerNl : public nodelet::Nodelet {
 public:
  Ll2GlobalPlannerNl();

 private:
  // Init
  virtual void onInit();
  void loadParams();

  // Subscriber callbacks
  void laneletMapCb(const autoware_lanelet2_msgs::MapBin& map_msg);
  void poseGoalCb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

  // Utility functions
  lanelet::Lanelet getNearestLanelet(const lanelet::BasicPoint2d& point);
  void planRoute(const geometry_msgs::Point& goal_point);

  // Nodehandles, both public and private
  ros::NodeHandle nh_, pnh_;

  // Publishers
  ros::Publisher waypoints_pub_;

  // Subscribers
  ros::Subscriber lanelet_sub_;
  ros::Subscriber posegoal_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Internal state
  bool initialized_ = false;
  lanelet::LaneletMapPtr lanelet_map_ = nullptr;
};

}  // namespace ll2_global_planner
#endif  // ll2_global_planner_Ll2GlobalPlannerNl_H
