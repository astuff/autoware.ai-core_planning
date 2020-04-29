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

#include <cstdio>
#include <memory>
#include <random>
#include <string>

#include <ros/ros.h>
#include <ros/spinner.h>

#include <autoware_msgs/Lane.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>

// lib
#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

namespace decision_maker
{

DecisionMakerNode::DecisionMakerNode()
  : private_nh_("~"),
  auto_mission_reload_(false),
  auto_engage_(false),
  auto_mission_change_(false),
  use_fms_(false),
  ignore_map_(false),
  insert_stop_line_wp_(true),
  param_num_of_steer_behind_(30),
  distance_before_lane_change_signal_(30),
  change_threshold_dist_(1.0),
  change_threshold_angle_(15),
  goal_threshold_dist_(3.0),
  goal_threshold_vel_(0.1),
  stopped_vel_(0.1),
  stopline_reset_count_(20),
  sim_mode_(false),
  use_lanelet_map_(false)
{
  std::string file_name_mission;
  std::string file_name_vehicle;
  std::string file_name_behavior;
  std::string file_name_motion;
  private_nh_.getParam("state_vehicle_file_name", file_name_vehicle);
  private_nh_.getParam("state_mission_file_name", file_name_mission);
  private_nh_.getParam("state_behavior_file_name", file_name_behavior);
  private_nh_.getParam("state_motion_file_name", file_name_motion);
  private_nh_.getParam("auto_mission_reload", auto_mission_reload_);
  private_nh_.getParam("auto_engage", auto_engage_);
  private_nh_.getParam("auto_mission_change", auto_mission_change_);
  private_nh_.getParam("use_fms", use_fms_);
  private_nh_.getParam("ignore_map", ignore_map_);
  private_nh_.getParam("param_num_of_steer_behind", param_num_of_steer_behind_);
  private_nh_.getParam("distance_before_lane_change_signal", distance_before_lane_change_signal_);
  private_nh_.getParam("change_threshold_dist", change_threshold_dist_);
  private_nh_.getParam("change_threshold_angle", change_threshold_angle_);
  private_nh_.getParam("goal_threshold_dist", goal_threshold_dist_);
  private_nh_.getParam("goal_threshold_vel", goal_threshold_vel_);
  private_nh_.getParam("stopped_vel", stopped_vel_);
  private_nh_.getParam("stopline_reset_count", stopline_reset_count_);
  private_nh_.getParam("sim_mode", sim_mode_);
  private_nh_.getParam("use_ll2", use_lanelet_map_);
  private_nh_.param<std::string>("stop_sign_id", stop_sign_id_, "stop_sign");

  current_status_.prev_stopped_wpidx = -1;

  ctx_vehicle.reset(new state_machine::StateContext(file_name_vehicle, "autoware_states_vehicle"));
  ctx_mission.reset(new state_machine::StateContext(file_name_mission, "autoware_states_mission"));
  ctx_behavior.reset(new state_machine::StateContext(file_name_behavior, "autoware_states_behavior"));
  ctx_motion.reset(new state_machine::StateContext(file_name_motion, "autoware_states_motion"));
  initROS();
  setupStateCallback();
}

void DecisionMakerNode::tryNextState(cstring_t& key)
{
  ctx_vehicle->nextState(key);
  ctx_mission->nextState(key);
  ctx_behavior->nextState(key);
  ctx_motion->nextState(key);
}

void DecisionMakerNode::update(void)
{
  update_msgs();
  if (ctx_vehicle)
    ctx_vehicle->onUpdate();
  if (ctx_mission)
    ctx_mission->onUpdate();
  if (ctx_behavior)
    ctx_behavior->onUpdate();
  if (ctx_motion)
    ctx_motion->onUpdate();
}

void DecisionMakerNode::run(void)
{
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    update();

    loop_rate.sleep();
  }
}

}  // namespace decision_maker
