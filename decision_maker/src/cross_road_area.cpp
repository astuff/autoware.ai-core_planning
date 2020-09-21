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
#include <tuple>
#include <algorithm>

#include <amathutils_lib/amathutils.hpp>

namespace decision_maker
{

#define TARGET_WAYPOINTS_NUM 15  // TODO(someone): need to change rosparam
constexpr double MAXIMUM_ANGLE_ERROR_THRESHOLD = 0.52;

CrossRoadArea::CrossRoadArea()
  : id{},
    area_id{},
    points{},
    bbox{},
    insideLanes{},
    insideWaypoints{}
{
}

std::vector<geometry_msgs::Point> CrossRoadArea::convhull(const CrossRoadArea* _TargetArea)
{
  // The convex hull of a set of points is defined as the smallest convex polygon,
  // that encloses all of the points in the set
  std::vector<int> enablePoints;

  if (_TargetArea->points.size() < 3)
    return {};

  // Jarvis's March algorithm
  size_t l = 0;
  for (auto i = begin(_TargetArea->points); i != end(_TargetArea->points); i++)
  {
    if (i->x < _TargetArea->points.at(l).x)
    {
      l = std::distance(begin(_TargetArea->points), i);
    }
  }

  size_t p = l;
  size_t q;

  do
  {
    q = (p + 1) % _TargetArea->points.size();
    for (size_t i = 0; i < _TargetArea->points.size(); i++)
    {
      geometry_msgs::Point pp = _TargetArea->points.at(p);
      geometry_msgs::Point pi = _TargetArea->points.at(i);
      geometry_msgs::Point pq = _TargetArea->points.at(q);
      if (((pi.y - pp.y) * (pq.x - pi.x) - (pi.x - pp.x) * (pq.y - pi.y)) < 0)
      {
        q = i;
      }
    }
    enablePoints.push_back(q);
    p = q;
  }
  while (p != l);

  std::vector<geometry_msgs::Point> point_arrays;
  for (auto p = begin(_TargetArea->points); p != end(_TargetArea->points); p++)
  {
    for (auto& en : enablePoints)
    {
      if (std::distance(begin(_TargetArea->points), p) == en)
      {
        point_arrays.push_back(*p);
      }
    }
  }

  // find mean x and y of all convhull points
  double x_mean = 0.0;
  double y_mean = 0.0;
  for (auto const point : point_arrays)
  {
    x_mean += point.x;
    y_mean += point.y;
  }
  x_mean = x_mean/point_arrays.size();
  y_mean = y_mean/point_arrays.size();

  // sort by ordering points clockwise with respect to average center point
  std::sort(point_arrays.begin(), point_arrays.end(),
    [x_mean, y_mean](const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs)
  {
    return std::atan2((lhs.x-x_mean), (lhs.y-y_mean)) < std::atan2((rhs.x-x_mean), (rhs.y-y_mean));
  });  //NOLINT

  return point_arrays;
}

bool CrossRoadArea::isInsideArea(const CrossRoadArea* _TargetArea, geometry_msgs::Point pt)
{
  std::vector<geometry_msgs::Point> point_arrays = convhull(_TargetArea);

  double rad = 0.0;
  for (auto it = begin(point_arrays); it != end(point_arrays); ++it)
  {
    auto it_n = it + 1;
    if (it == --point_arrays.end())
    {
      it_n = point_arrays.begin();
    }

    double ax = it->x - pt.x;
    double ay = it->y - pt.y;
    double bx = it_n->x - pt.x;
    double by = it_n->y - pt.y;
    double cos_ = (ax * bx + ay * by) / (sqrt(ax * ax + ay * ay) * sqrt(bx * bx + by * by));
    if (cos_ > 1.0)
    {
      cos_ = 1;
    }
    else if (cos_ < -1.0)
    {
      cos_ = -1.0;
    }

    // deg += std::acos(cos_)? std::acos(cos_)/ M_PI * 180.0 : 0.0;
    rad += std::acos(cos_) ? std::acos(cos_) : 0.0;
  }
  if (fabs((2 * M_PI) - rad) <= MAXIMUM_ANGLE_ERROR_THRESHOLD /*about 30 degree*/)
  {
    return true;
  }

  return false;
}

}  // namespace decision_maker
