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

#include <boost_geometry_util/geometries/polygon.hpp>

namespace boost_geometry_util
{
boost::geometry::model::polygon<boost_geometry_util::Point2D> toPolygon(
  const std::vector<geometry_msgs::msg::Point> & linestring)
{
  boost::geometry::model::polygon<boost_geometry_util::Point2D> poly;
  std::for_each(linestring.begin(), linestring.end(), [&poly](const auto & point) {
    boost::geometry::exterior_ring(poly).emplace_back(
      boost_geometry_util::Point2D(point.x, point.y));
  });
  return poly;
}

boost::geometry::model::polygon<boost_geometry_util::Point2D> toPolygon(
  const std::vector<boost_geometry_util::Point2D> & linestring)
{
  boost::geometry::model::polygon<boost_geometry_util::Point2D> poly;
  std::for_each(linestring.begin(), linestring.end(), [&poly](const auto & point) {
    boost::geometry::exterior_ring(poly).emplace_back(point);
  });
  return poly;
}
}  // namespace boost_geometry_util
