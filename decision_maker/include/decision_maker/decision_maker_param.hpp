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

#ifndef DECISION_MAKER_DECISION_MAKER_PARAM_HPP
#define DECISION_MAKER_DECISION_MAKER_PARAM_HPP

namespace decision_maker
{
constexpr bool DEFAULT_DISPLAY_FLAG = false;
constexpr bool DEFAULT_FORCE_STATE_CHANGE = true;

constexpr int VEL_AVERAGE_COUNT = 10;
constexpr int DEFAULT_CONVERGENCE_COUNT = 5;
constexpr double DEFAULT_CONVERGENCE_THRESHOLD = 0.01;

constexpr double DOUBLE_MAX = 1.7976931348623158e308;
constexpr int DEFAULT_TARGET_WAYPOINT = 14;

constexpr double DEFAULT_STOP_DECELERATION = 0.2;
constexpr int DEFAULT_STOP_TIME = 1;
constexpr int DEFAULT_STOPLINE_TARGET_WAYPOINT = 10;
constexpr double DEFAULT_STOPLINE_TARGET_RATIO = 0.0;
constexpr double DEFAULT_SHIFT_WIDTH = 1.0;
constexpr double DEFAULT_CRAWL_VELOCITY = 4.0;
constexpr double DEFAULT_DETECTION_AREA_RATE = 1.0;

constexpr double ANGLE_NEUTRAL = 0;
constexpr double ANGLE_CURVE = 40;
constexpr double ANGLE_LEFT = (ANGLE_NEUTRAL - ANGLE_CURVE);
constexpr double ANGLE_RIGHT = (ANGLE_NEUTRAL + ANGLE_CURVE);

constexpr char TPNAME_BASED_LANE_WAYPOINTS_ARRAY[] = "/based/lane_waypoints_array";
constexpr char TPNAME_CONTROL_LANE_WAYPOINTS_ARRAY[] = "/lane_waypoints_array";
constexpr int LAMP_ON = 1;
constexpr int LAMP_OFF = 0;

constexpr double DEFAULT_DETECTION_AREA_X1 = 80.0;
constexpr double DEFAULT_DETECTION_AREA_X2 = 0.0;
constexpr double DEFAULT_DETECTION_AREA_Y1 = 50.0;
constexpr double DEFAULT_DETECTION_AREA_Y2 = -50.0;
}  // namespace decision_maker

#endif  // DECISION_MAKER_DECISION_MAKER_PARAM_HPP
