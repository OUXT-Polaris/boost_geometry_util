// Copyright (c) 2022 OUXT Polaris
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

#ifndef BOOST_GEOMETRY_UTIL__GEOMETRIES__POINT_HPP_
#define BOOST_GEOMETRY_UTIL__GEOMETRIES__POINT_HPP_

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace boost_geometry_util
{
class Point2D
{
public:
  explicit Point2D(const geometry_msgs::msg::Point & point);
  explicit Point2D(double x, double y);
  Point2D() = default;
  double x;
  double y;
};
}  // namespace boost_geometry_util

BOOST_GEOMETRY_REGISTER_POINT_2D(geometry_msgs::msg::Point, double, cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_POINT_2D(boost_geometry_util::Point2D, double, cs::cartesian, x, y)

#endif  // BOOST_GEOMETRY_UTIL__GEOMETRIES__POINT_HPP_
