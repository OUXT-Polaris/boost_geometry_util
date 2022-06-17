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

// headers in Google Test
#include <gtest/gtest.h>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost_geometry_util/geometries/geometries.hpp>

namespace bg = boost::geometry;

#define EXPECT_POINT2D_EQ(POINT, X, Y)                 \
  EXPECT_DOUBLE_EQ(boost::geometry::get<0>(POINT), X); \
  EXPECT_DOUBLE_EQ(boost::geometry::get<1>(POINT), Y);

#define EXPECT_BOX2D_EQ(BOX, MIN_CORNER_X, MIN_CORNER_Y, MAX_CORNER_X, MAX_CORNER_Y) \
  EXPECT_POINT2D_EQ(BOX.min_corner, MIN_CORNER_X, MIN_CORNER_Y);                     \
  EXPECT_POINT2D_EQ(BOX.max_corner, MAX_CORNER_X, MAX_CORNER_Y);

TEST(TestSuite, Point2D)
{
  const auto point = boost_geometry_util::Point2D(3, 5);
  EXPECT_POINT2D_EQ(point, 3, 5);
  geometry_msgs::msg::Point ros_point;
  {
    ros_point.x = 1.0;
    ros_point.y = 2.0;
    ros_point.z = 0.3;
  };
  EXPECT_POINT2D_EQ(ros_point, 1, 2);
}

TEST(TestSuite, Box)
{
  const auto b0 = boost_geometry_util::Box2D(
    boost_geometry_util::Point2D(0, 0), boost_geometry_util::Point2D(3, 3));
  const auto b1 = boost_geometry_util::Box2D(
    boost_geometry_util::Point2D(4, 4), boost_geometry_util::Point2D(7, 7));
  EXPECT_BOX2D_EQ(b0, 0, 0, 3, 3);
  EXPECT_BOX2D_EQ(b1, 4, 4, 7, 7);
  geometry_msgs::msg::Point ros_point0;
  {
    ros_point0.x = 0.0;
    ros_point0.y = 0.0;
    ros_point0.z = 0.3;
  };
  geometry_msgs::msg::Point ros_point1;
  {
    ros_point1.x = 3.0;
    ros_point1.y = 3.0;
    ros_point1.z = 0.3;
  };
  const auto b2 = boost_geometry_util::Box2D(ros_point0, ros_point1);
  EXPECT_BOX2D_EQ(b2, 0, 0, 3, 3);
}

TEST(TestSuite, LineString)
{
  std::vector<geometry_msgs::msg::Point> line0;
  {
    geometry_msgs::msg::Point p0, p1;
    p0.x = 0;
    p0.y = 2;
    p1.x = 2;
    p1.y = 2;
    line0.emplace_back(p0);
    line0.emplace_back(p1);
  }
  std::vector<geometry_msgs::msg::Point> line1;
  {
    geometry_msgs::msg::Point p0, p1;
    p0.x = 1;
    p0.y = 0;
    p1.x = 1;
    p1.y = 4;
    line1.emplace_back(p0);
    line1.emplace_back(p1);
  }
  EXPECT_TRUE(bg::intersects(line0, line1));
  std::vector<geometry_msgs::msg::Point> line2;
  {
    geometry_msgs::msg::Point p0, p1;
    p0.x = 0;
    p0.y = 0;
    p1.x = 2;
    p1.y = 0;
    line0.emplace_back(p0);
    line0.emplace_back(p1);
  }
  std::vector<geometry_msgs::msg::Point> line3;
  {
    geometry_msgs::msg::Point p0, p1;
    p0.x = 0;
    p0.y = 2;
    p1.x = 2;
    p1.y = 2;
    line1.emplace_back(p0);
    line1.emplace_back(p1);
  }
  EXPECT_FALSE(bg::intersects(line2, line3));
}

TEST(TestSuite, Polygon)
{
  boost_geometry_util::toPolygon(std::vector<boost_geometry_util::Point2D>(
    {boost_geometry_util::Point2D(2.0, 1.3), boost_geometry_util::Point2D(2.4, 1.7),
     boost_geometry_util::Point2D(3.6, 1.2), boost_geometry_util::Point2D(4.6, 1.6),
     boost_geometry_util::Point2D(4.1, 3.0), boost_geometry_util::Point2D(5.3, 2.8),
     boost_geometry_util::Point2D(5.4, 1.2), boost_geometry_util::Point2D(4.9, 0.8),
     boost_geometry_util::Point2D(3.6, 0.7), boost_geometry_util::Point2D(2.0, 1.3)}));
  geometry_msgs::msg::Point p0, p1;
  p0.x = 1;
  p0.y = 0;
  p1.x = 1;
  p1.y = 4;
  boost_geometry_util::toPolygon(std::vector<geometry_msgs::msg::Point>({p0, p1}));
}

TEST(TestSuite, Disjoint)
{
  const auto b0 = boost_geometry_util::Box2D(
    boost_geometry_util::Point2D(0, 0), boost_geometry_util::Point2D(3, 3));
  const auto b1 = boost_geometry_util::Box2D(
    boost_geometry_util::Point2D(4, 4), boost_geometry_util::Point2D(7, 7));
  EXPECT_TRUE(bg::disjoint(b0, b1));
  const auto b2 = boost_geometry_util::Box2D(
    boost_geometry_util::Point2D(2, 2), boost_geometry_util::Point2D(5, 5));
  EXPECT_FALSE(bg::disjoint(b0, b2));
  const auto p0 = boost_geometry_util::Point2D(4, 4);
  EXPECT_TRUE(bg::disjoint(b0, p0));
  const auto p1 = boost_geometry_util::Point2D(2, 2);
  EXPECT_FALSE(bg::disjoint(b0, p1));
  geometry_msgs::msg::Point ros_point;
  {
    ros_point.x = 2.0;
    ros_point.y = 2.0;
    ros_point.z = 0.3;
  };
  EXPECT_FALSE(bg::disjoint(b0, ros_point));
}

TEST(TestSuite, Area)
{
  const auto b0 = boost_geometry_util::Box2D(
    boost_geometry_util::Point2D(0, 0), boost_geometry_util::Point2D(3, 3));
  EXPECT_DOUBLE_EQ(bg::area(b0), 9);
}

TEST(TestSuite, ConvexHull)
{
  std::vector<boost_geometry_util::Point2D> linestring = {
    boost_geometry_util::Point2D(2.0, 1.3), boost_geometry_util::Point2D(2.4, 1.7),
    boost_geometry_util::Point2D(3.6, 1.2), boost_geometry_util::Point2D(4.6, 1.6),
    boost_geometry_util::Point2D(4.1, 3.0), boost_geometry_util::Point2D(5.3, 2.8),
    boost_geometry_util::Point2D(5.4, 1.2), boost_geometry_util::Point2D(4.9, 0.8),
    boost_geometry_util::Point2D(3.6, 0.7), boost_geometry_util::Point2D(2.0, 1.3)};
  /*
  bg::model::polygon<boost_geometry_util::Point2D> poly = boost_geometry_util::toPolygon();
  bg::model::polygon<boost_geometry_util::Point2D> hull;
  bg::convex_hull(poly, hull);
  */
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
