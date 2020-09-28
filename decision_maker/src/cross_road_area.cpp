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

#include "decision_maker/cross_road_area.h"

#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

#include <amathutils_lib/amathutils.hpp>

namespace decision_maker
{
constexpr double MAXIMUM_ANGLE_ERROR_THRESHOLD = 0.52;

int CrossRoadArea::findOrientationPoints(const geometry_msgs::Point& ptA,
                                         const geometry_msgs::Point& ptB,
                                         const geometry_msgs::Point& ptC)
{
  return (ptB.y - ptA.y) * (ptC.x - ptB.x) - (ptB.x - ptA.x) * (ptC.y - ptB.y);
}

void CrossRoadArea::calcIntersectionBBox()
{
  if (points.size() < 2)
    return;

  // find the centroid poisition of the intersection
  bbox.pose.position.x = 0.0;
  bbox.pose.position.y = 0.0;
  bbox.pose.position.z = 0.0;
  for (const auto& point : points)
  {
    bbox.pose.position.x += point.x;
    bbox.pose.position.y += point.y;
    bbox.pose.position.z += point.z;
  }
  bbox.pose.position.x = bbox.pose.position.x / points.size();
  bbox.pose.position.y = bbox.pose.position.y / points.size();
  bbox.pose.position.z = bbox.pose.position.z / points.size();

  // find the estimate dimension of the intersection
  double x_min = 0.0, x_max = 0.0;
  double y_min = 0.0, y_max = 0.0;
  for (const auto& point : points)
  {
    x_min = (x_min == 0.0) ? point.x : std::min(point.x, x_min);
    x_max = (x_max == 0.0) ? point.x : std::max(point.x, x_max);
    y_min = (y_min == 0.0) ? point.y : std::min(point.y, y_min);
    y_max = (y_max == 0.0) ? point.y : std::max(point.y, y_max);
  }

  // dimensions of intersection bbox
  bbox.dimensions.x = x_max - x_min;
  bbox.dimensions.y = y_max - y_min;
  bbox.dimensions.z = 2;
  bbox.label = 1;
}

void CrossRoadArea::convhull()
{
  // points array needs to have at least three points
  if (points.size() < 3)
    return;

  // find the leftmost point
  auto leftmost_iter = std::min_element(points.begin(), points.end(),
    [] (const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2)
    {
        return pt1.x < pt2.x;
    });  //NOLINT
  size_t leftmost_pt_idx = leftmost_iter - points.begin();

  // Jarvis's March algorithm
  // Start from leftmost point, keep moving counterclockwise until reach the start point again
  size_t p = leftmost_pt_idx;
  size_t q;
  do
  {
    // Search for a point 'q' such that orientation is counterclockwise for all points 'x'.
    // The idea is to keep track of last visited most counterclock-wise point in q.
    // If any point 'i' is more counterclock-wise than q, then update q.
    q = (p + 1) % points.size();
    for (size_t i = 0; i < points.size(); i++)
    {
      // solving for orientation [result < 0 : ccw], [result > 0 : cw], [result = 0 : collinear]
      if (findOrientationPoints(points.at(p), points.at(i), points.at(q)) < 0)
      {
        // If i is more counterclockwise than current q, then update q
        q = i;
      }
    }
    convhull_points.push_back(points.at(q));
    // At this point, q is the most counterclockwise with respect to p
    // Set p as q for next iteration, so that q is added to result 'convhull_idx'
    p = q;
  }
  while (p != leftmost_pt_idx);

  // find mean x and y of all convhull points
  double x_mean = 0.0;
  double y_mean = 0.0;
  for (auto const point : convhull_points)
  {
    x_mean += point.x;
    y_mean += point.y;
  }
  x_mean = x_mean/convhull_points.size();
  y_mean = y_mean/convhull_points.size();

  // sort by ordering points clockwise with respect to average center point
  std::sort(convhull_points.begin(), convhull_points.end(),
    [x_mean, y_mean](const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs)
  {
    return std::atan2((lhs.x-x_mean), (lhs.y-y_mean)) < std::atan2((rhs.x-x_mean), (rhs.y-y_mean));
  });  //NOLINT

  return;
}

bool CrossRoadArea::addWaypoint(const autoware_msgs::Waypoint& wp)
{
  geometry_msgs::Point pp;
  pp.x = wp.pose.pose.position.x;
  pp.y = wp.pose.pose.position.y;
  pp.z = wp.pose.pose.position.z;

  if (isPointInsideCrossRoadArea(pp))
  {
    if (inside_lanes.empty() || wp.gid != inside_lanes.back().waypoints.back().gid + 1)
    {
      inside_lanes.emplace_back();
      bbox.pose.orientation = wp.pose.pose.orientation;
    }
    inside_lanes.back().waypoints.push_back(wp);  // autoware_msgs::Lane
    inside_waypoints.push_back(wp);  // autoware_msgs::Waypoint

    return true;
  }
  return false;
}

bool CrossRoadArea::isPointInsideArea(const geometry_msgs::Point& point,
                                      const std::vector<geometry_msgs::Point>& area_points)
{
  // compute the sum of the angles between a given point and every neighboring pair of edge points
  // which should equal to 360 degrees or 2 pi on a 2D plane
  double sum_theta = 0.0;
  for (auto it = begin(area_points); it != end(area_points); ++it)
  {
    auto it_n = it + 1;
    if (it == --area_points.end())
    {
      it_n = area_points.begin();
    }

    const double ax = it->x - point.x;
    const double ay = it->y - point.y;
    const double bx = it_n->x - point.x;
    const double by = it_n->y - point.y;

    // if the point equals as any of the area_points, include it
    if ((ax == 0 && ay == 0) || (bx == 0 && by == 0))
    {
      return true;
    }

    double cos_theta = (ax * bx + ay * by) / (sqrt(ax * ax + ay * ay) * sqrt(bx * bx + by * by));

    const double theta = std::acos(cos_theta);
    if (isnan(theta))
    {
      continue;
    }
    else
    {
      sum_theta += theta;
    }
  }
  if (fabs((2 * M_PI) - sum_theta) <= MAXIMUM_ANGLE_ERROR_THRESHOLD /*about 30 degree*/)
  {
    return true;
  }

  return false;
}

bool CrossRoadArea::isPointInsideCrossRoadArea(const geometry_msgs::Point& pt)
{
  return isPointInsideArea(pt, convhull_points);
}

bool CrossRoadArea::tryLookupTransform(geometry_msgs::TransformStamped &tf_local2global,
                                       const std::string global_frame,
                                       const std::string local_frame)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  int num_of_tries = 0;
  while (1)
  {
    try
    {
      tf_local2global = tfBuffer.lookupTransform(global_frame, local_frame, ros::Time(0), ros::Duration(1.0));
      return true;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      if (++num_of_tries == tf_max_number_of_tries)
      {
        return false;
      }
    }
  }
}

bool CrossRoadArea::tryDoTransform(const geometry_msgs::Pose object_local_pose,
                                   geometry_msgs::Pose &obj_global_pose,
                                   const geometry_msgs::TransformStamped tf_local2global)
{
  try
  {
    tf2::doTransform(object_local_pose, obj_global_pose, tf_local2global);
    return true;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool CrossRoadArea::isObjectInsideStopAreas(const autoware_msgs::DetectedObjectArray& msg, bool init_flag)
{
  geometry_msgs::TransformStamped tf_local2global;
  std::string local_frame = msg.header.frame_id;
  const std::string global_frame = "map";

  if (!tryLookupTransform(tf_local2global, global_frame, local_frame))
  {
    ROS_ERROR("Transform retry attempt reached max_num_of_tries! Resetting stopline_safety_timer");
    return true;
  }

  int detect_total_count = 0;
  for (auto& stop_area : stop_areas)
  {
    if (stop_area.is_safe == -1)
    {
      continue;
    }
    int detect_count = 0;
    for (const auto& obj : msg.objects)
    {
      geometry_msgs::Pose obj_global_pose;
      if (!tryDoTransform(obj.pose, obj_global_pose, tf_local2global))
      {
        ROS_ERROR("Transform retry attempt reached max_num_of_tries! Resetting stopline_safety_timer");
        return true;
      }
      // if detected obj is inside the zone defined by roi_points, not safe to enter intersection
      if (isPointInsideArea(obj_global_pose.position, stop_area.roi_points))
      {
        ++detect_count;
      }
    }

    ROS_DEBUG("stopline: %d | roi_points: (%f , %f) (%f , %f) | is_safe %d, detect_count: %d",
              stop_area.stopline_id,
              stop_area.roi_points.front().x, stop_area.roi_points.front().y,
              stop_area.roi_points.back().x, stop_area.roi_points.back().y,
              stop_area.is_safe, detect_count);

    if (detect_count == 0)
    {
      // empty stop areas when ego vehicle stops at the intersection are ignored as ego vehicle has priority
      if (!init_flag)
      {
        stop_area.is_safe = -1;
      }
      else
      {
        stop_area.is_safe = 1;
      }
    }
    else
    {
      stop_area.is_safe = 0;
      ++detect_total_count;
    }
  }

  return (detect_total_count != 0);
}

void CrossRoadArea::addStopArea(const int stopline_id, const geometry_msgs::Point &ptA,
                                const geometry_msgs::Point &ptB, const int stopline_detect_dist_)
{
  CrossRoadArea::StopArea stop_area;
  stop_area.stopline_id = stopline_id;
  stop_area.roi_points = createRectagularStopAreaFromStopLine(ptA, ptB, stopline_detect_dist_);
  stop_areas.push_back(stop_area);

  ROS_DEBUG("intersection_id: %d | stopline.id: %d | roi_points: (%f , %f) (%f , %f)\n", id, stopline_id,
                                   stop_area.roi_points.front().x, stop_area.roi_points.front().y,
                                   stop_area.roi_points.back().x, stop_area.roi_points.back().y);
}

std::vector<geometry_msgs::Point> CrossRoadArea::createRectagularStopAreaFromStopLine(
                                  const geometry_msgs::Point &ptA, const geometry_msgs::Point &ptB,
                                  const double length)
{
  /*********************************************************************
  Find C(x,y) and D(x,y)
            * Exterior Point (intersection's centroid point)
      A *------* B
        |      |
      C *------* D
  AB : stopline
  ABCD : area of interest
  known : A(x,y), B(x,y), AC and BD (length), point outside (exterior)
  length of AC and BD are defined by us
  *********************************************************************/
  const geometry_msgs::Point ptExt = bbox.pose.position;  // Exterior Point
  const double AC = length;
  geometry_msgs::Point ptC, ptD;

  // avoid division-by-zero case when finding slope
  if (ptA.x == ptB.x)
  {
    // if the line is vertical, shift the line by adding/subtracting the length
    geometry_msgs::Point ptC_1, ptC_2;
    ptC_1.x = ptA.x + AC;
    ptC_2.x = ptA.x - AC;
    ptC.x = (fabs(ptC_1.x - ptExt.x) > fabs(ptC_2.x - ptExt.x)) ? ptC_1.x : ptC_2.x;
    ptD.x = ptC.x;
    ptC.y = ptA.y;
    ptD.y = ptB.y;
  }
  // avoid division-by-zero case when finding slope
  else if (ptA.y == ptB.y)
  {
    // if the line is horizontal, shift the line by adding/subtracting the length
    geometry_msgs::Point ptC_1, ptC_2;
    ptC_1.y = ptA.y + AC;
    ptC_2.y = ptA.y - AC;
    ptC.y = (fabs(ptC_1.y - ptExt.y) > fabs(ptC_2.y - ptExt.y)) ? ptC_1.y : ptC_2.y;
    ptD.y = ptC.y;
    ptC.x = ptA.x;
    ptD.x = ptB.x;
  }
  else
  {
    // slope of the two stop points
    double m_orig = (ptB.y-ptA.y)/(ptB.x-ptA.x);
    // perpendicular from the stopline
    double m = -(1/m_orig);

    // complete the equation of a line
    // y = mx + b or b = y - mx
    double b_c = ptA.y - m*ptA.x;

    // find the point C which is on the perpendicular line with distance `length` away from the point A
    // numerical coefficients for quadratic formula
    // a = m^2 + 1
    // b = 2(m)(b_c) - 2(m)(y_a) - 2(x_a)
    // c = (b_c)^2 - 2(y_a)(b_c) + (y_a)^2 - AC^2
    double a = m*m + 1;
    double b = 2*m*b_c - 2*m*ptA.y - 2*ptA.x;
    double c = b_c*b_c - 2*ptA.y*b_c + ptA.y*ptA.y + ptA.x*ptA.x - AC*AC;

    geometry_msgs::Point ptC_1, ptC_2;
    // quadratic formula
    ptC_1.x = (-b + sqrt(b*b - 4*a*c)) / (2*a);
    ptC_2.x = (-b - sqrt(b*b - 4*a*c)) / (2*a);
    // find y
    ptC_1.y = m*ptC_1.x + b_c;
    ptC_2.y = m*ptC_2.x + b_c;
    // choose the pair that are farther away from the exterior point
    ptC = (amathutils::find_distance(ptExt, ptC_1) > amathutils::find_distance(ptExt, ptC_2)) ? ptC_1 : ptC_2;
    // apply offset between point A and B to calculate point D
    ptD.x = ptC.x + (ptB.x - ptA.x);
    ptD.y = ptC.y + (ptB.y - ptA.y);
  }
  // return all points
    return std::vector<geometry_msgs::Point>{ptA, ptB, ptD, ptC};
}

}  // namespace decision_maker
