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
  const auto point = boost_geometry_utils::Point2D(3, 5);
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
  const auto b0 = boost_geometry_utils::Box2D(
    boost_geometry_utils::Point2D(0, 0), boost_geometry_utils::Point2D(3, 3));
  const auto b1 = boost_geometry_utils::Box2D(
    boost_geometry_utils::Point2D(4, 4), boost_geometry_utils::Point2D(7, 7));
  EXPECT_BOX2D_EQ(b0, 0, 0, 3, 3);
  EXPECT_BOX2D_EQ(b1, 4, 4, 7, 7);
}

TEST(TestSuite, Disjoint)
{
  const auto b0 = boost_geometry_utils::Box2D(
    boost_geometry_utils::Point2D(0, 0), boost_geometry_utils::Point2D(3, 3));
  const auto b1 = boost_geometry_utils::Box2D(
    boost_geometry_utils::Point2D(4, 4), boost_geometry_utils::Point2D(7, 7));
  EXPECT_TRUE(bg::disjoint(b0, b1));
  const auto b2 = boost_geometry_utils::Box2D(
    boost_geometry_utils::Point2D(2, 2), boost_geometry_utils::Point2D(5, 5));
  EXPECT_FALSE(bg::disjoint(b0, b2));
  const auto p0 = boost_geometry_utils::Point2D(4, 4);
  EXPECT_TRUE(bg::disjoint(b0, p0));
  const auto p1 = boost_geometry_utils::Point2D(2, 2);
  EXPECT_FALSE(bg::disjoint(b0, p1));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
