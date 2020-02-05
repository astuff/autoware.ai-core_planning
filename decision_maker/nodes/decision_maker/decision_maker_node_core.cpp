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

#include "decision_maker_node.hpp"

#include <cstdio>
#include <random>

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
