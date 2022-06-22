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
#include <boost_geometry_util/geometries/geometries.hpp>

namespace bg = boost::geometry;

#define EXPECT_POINT2D_EQ(POINT, X, Y)                 \
  EXPECT_DOUBLE_EQ(boost::geometry::get<0>(POINT), X); \
  EXPECT_DOUBLE_EQ(boost::geometry::get<1>(POINT), Y);

#define EXPECT_POINT3D_EQ(POINT, X, Y, Z)              \
  EXPECT_DOUBLE_EQ(boost::geometry::get<0>(POINT), X); \
  EXPECT_DOUBLE_EQ(boost::geometry::get<1>(POINT), Y); \
  EXPECT_DOUBLE_EQ(boost::geometry::get<2>(POINT), Z);

#define EXPECT_BOX2D_EQ(BOX, MIN_CORNER_X, MIN_CORNER_Y, MAX_CORNER_X, MAX_CORNER_Y) \
  EXPECT_POINT2D_EQ(BOX.min_corner, MIN_CORNER_X, MIN_CORNER_Y);                     \
  EXPECT_POINT2D_EQ(BOX.max_corner, MAX_CORNER_X, MAX_CORNER_Y);

#define TEST_POINT_TYPE_FOREACH(IDENTIFIER, ...)      \
  IDENTIFIER<geometry_msgs::msg::Point>(__VA_ARGS__); \
  IDENTIFIER<geometry_msgs::msg::Point32>(__VA_ARGS__);

template <typename T>
void testPoint2D(double x, double y)
{
  EXPECT_POINT2D_EQ(boost_geometry_util::point_2d::construct<T>(x, y), x, y);
}

template <typename T>
void testPoint2DApply(double x, double y, double vec_x, double vec_y)
{
  EXPECT_POINT2D_EQ(
    boost_geometry_util::point_2d::construct<T>(x, y) +
      boost_geometry_util::vector_2d::construct(vec_x, vec_y),
    x + vec_x, y + vec_y);
}

template <typename T>
void testPoint2DSubtract(double x, double y, double vec_x, double vec_y)
{
  EXPECT_POINT2D_EQ(
    boost_geometry_util::point_2d::construct<T>(x, y) -
      boost_geometry_util::vector_2d::construct(vec_x, vec_y),
    x - vec_x, y - vec_y);
}

TEST(TestSuite, Point2D)
{
  TEST_POINT_TYPE_FOREACH(testPoint2D, 1.0, 2.0);
  TEST_POINT_TYPE_FOREACH(testPoint2DApply, 1.0, 2.0, 3, 4);
  TEST_POINT_TYPE_FOREACH(testPoint2DSubtract, 1.0, 2.0, 3, 4);
}

template <typename T>
void testPoint3D(double x, double y, double z)
{
  EXPECT_POINT3D_EQ(boost_geometry_util::point_3d::construct<T>(x, y, z), x, y, z);
}

template <typename T>
void testPoint3DApply(double x, double y, double z, double vec_x, double vec_y, double vec_z)
{
  EXPECT_POINT3D_EQ(
    boost_geometry_util::point_3d::construct<T>(x, y, z) +
      boost_geometry_util::vector_3d::construct(vec_x, vec_y, vec_z),
    x + vec_x, y + vec_y, z + vec_z);
}

template <typename T>
void testPoint3DSubtract(double x, double y, double z, double vec_x, double vec_y, double vec_z)
{
  EXPECT_POINT3D_EQ(
    boost_geometry_util::point_3d::construct<T>(x, y, z) -
      boost_geometry_util::vector_3d::construct(vec_x, vec_y, vec_z),
    x - vec_x, y - vec_y, z - vec_z);
}

TEST(TestSuite, Point3D)
{
  TEST_POINT_TYPE_FOREACH(testPoint3D, 1.0, 2.0, 3.0);
  TEST_POINT_TYPE_FOREACH(testPoint3DApply, 1.0, 2.0, 3.0, 4, 5, 6);
  TEST_POINT_TYPE_FOREACH(testPoint3DSubtract, 1.0, 2.0, 3.0, 4, 5, 6);
}

template <typename T>
void testBox(double x_min, double y_min, double z_min, double x_max, double y_max, double z_max)
{
  EXPECT_NO_THROW(bg::model::box<T> box(
    boost_geometry_util::point_3d::construct<T>(x_min, y_min, z_min),
    boost_geometry_util::point_3d::construct<T>(x_max, y_max, z_max)));
}

TEST(TestSuite, Box) { TEST_POINT_TYPE_FOREACH(testBox, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0); }

template <typename T>
void testLineString(
  double x_begin, double y_begin, double z_begin, double x_end, double y_end, double z_end)
{
  const bg::model::linestring<T> line0 = {
    boost_geometry_util::point_3d::construct<T>(x_begin, y_begin, z_begin),
    boost_geometry_util::point_3d::construct<T>(x_end, y_end, z_end)};
  EXPECT_NO_THROW(bg::correct(line0));
}

TEST(TestSuite, LineString) { TEST_POINT_TYPE_FOREACH(testLineString, 0, 2, 0, 2, 2, 0); }

/*
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
  bg::model::polygon<boost_geometry_util::Point2D> poly = boost_geometry_util::toPolygon();
  bg::model::polygon<boost_geometry_util::Point2D> hull;
  bg::convex_hull(poly, hull);
}
*/

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
