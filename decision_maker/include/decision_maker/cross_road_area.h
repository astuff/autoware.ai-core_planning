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

#ifndef DECISION_MAKER_CROSS_ROAD_AREA_H
#define DECISION_MAKER_CROSS_ROAD_AREA_H

#include <vector>
#include <string>

#include <autoware_msgs/Lane.h>
#include <geometry_msgs/Point.h>
#include <vector_map_msgs/StopLine.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <tf2_ros/transform_listener.h>

namespace decision_maker
{
constexpr int tf_max_number_of_tries = 3;

class CrossRoadArea
{
public:
  /**
   * @brief finds centroid poisition and estimate dimension of the intersection
   */
  void calcIntersectionBBox();

  /**
   * @brief finds the smallest set of points that encloses all of the points in the set
   */
  void convhull();

  /**
   * @brief add to inside_lanes and inside_waypoints of an intersection if it's inside
   * @param waypoint
   * @return boolean
   */
  bool addWaypoint(const autoware_msgs::Waypoint& wp);

  /**
   * @brief determines if the given point is inside any of intersections from intersects_
   * @param point
   * @return boolean
   */
  bool isPointInsideCrossRoadArea(const geometry_msgs::Point& pt);

  /**
   * @brief determines if the given object is inside any of stop_areas from approaching intersection
   * @param object array
   * @return boolean
   */
  bool isObjectInsideStopAreas(const autoware_msgs::DetectedObjectArray& msg, bool init_flag);

  /**
   * @brief assign stop area to intersection
   * @param stopline point A
   * @param stopline point B
   * @param stopline_detect_dist_
   */
  void addStopArea(const int stopline_id, const geometry_msgs::Point &ptA,
                   const geometry_msgs::Point &ptB, const int stopline_detect_dist_);

  /**
   * @brief finds the two missing points of a rectangle drawn from two points and its length
   * @param point stopline_fp
   * @param point stopline_bp
   * @return vector of points that make up the rectangle
   */
  std::vector<geometry_msgs::Point> createRectagularStopAreaFromStopLine(const geometry_msgs::Point& stopline_fp,
                                                                         const geometry_msgs::Point& stopline_bp,
                                                                         const double length);

  int id = -1;
  int area_id = -1;
  std::vector<geometry_msgs::Point> points;
  std::vector<geometry_msgs::Point> convhull_points;
  jsk_recognition_msgs::BoundingBox bbox;
  std::vector<autoware_msgs::Lane> inside_lanes;
  std::vector<autoware_msgs::Waypoint> inside_waypoints;

  struct StopArea
  {
    std::vector<geometry_msgs::Point> roi_points{};
    int stopline_id{-1};
    int is_safe{0};  // [-1 = ignore], [0 = occupied / not safe], [1 = safe]
  };
  std::vector<StopArea> stop_areas;

private:
  /**
   * @brief try lookupTransform multiple times
   * @param tf_local2global
   * @param global_frame
   * @param local_frame
   * @return boolean
   */
  bool tryLookupTransform(geometry_msgs::TransformStamped &tf_local2global,
                          const std::string global_frame,
                          const std::string local_frame);


  /**
   * @brief try doTransform once
   * @param global_frame
   * @param point obj_global_pose
   * @param point local_frame
   * @return tf_local2global
   */
  bool tryDoTransform(const geometry_msgs::Pose global_frame,
                      geometry_msgs::Pose &obj_global_pose,
                      const geometry_msgs::TransformStamped tf_local2global);

  /**
   * @brief determine orientatinon of given points
   * @param point A
   * @param point B
   * @param point C
   * @return [result < 0 : ccw], [result > 0 : cw], [result = 0 : collinear]
   */
  int findOrientationPoints(const geometry_msgs::Point& ptA,
                                           const geometry_msgs::Point& ptB,
                                           const geometry_msgs::Point& ptC);

  /**
   * @brief determine if the given point is inside given area
   * @param point being examined
   * @param point vector of the area
   * @return boolean
   */
  bool isPointInsideArea(const geometry_msgs::Point& pt, const std::vector<geometry_msgs::Point>& area_points);
};
}  // namespace decision_maker

#endif  // DECISION_MAKER_CROSS_ROAD_AREA_H
