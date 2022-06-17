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

#ifndef BOOST_GEOMETRY_UTIL__GEOMETRIES__POLYGON_HPP_
#define BOOST_GEOMETRY_UTIL__GEOMETRIES__POLYGON_HPP_

#include <boost/geometry/geometries/polygon.hpp>
#include <boost_geometry_util/geometries/linestring.hpp>
#include <boost_geometry_util/geometries/point.hpp>

namespace boost_geometry_util
{
template <typename T>
boost::geometry::model::polygon<boost_geometry_util::Point2D> toPolygon(
  const std::vector<T> & linestring)
{
  boost::geometry::model::polygon<boost_geometry_util::Point2D> poly;
  std::for_each(linestring.begin(), linestring.end(), [&poly](const auto & point) {
    boost::geometry::exterior_ring(poly).emplace_back(
      boost_geometry_util::Point2D(point.x, point.y));
  });
  return poly;
}
}  // namespace boost_geometry_util

#endif  // BOOST_GEOMETRY_UTIL__GEOMETRIES__POINT_HPP_
