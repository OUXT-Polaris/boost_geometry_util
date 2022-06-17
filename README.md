# boost_geometry_util

boost geometry friendly library for ROS2.

## What we can do.

You can use boost geometry with ROS2 very easily.

```cpp
const auto b0 = boost_geometry_util::Box2D(
  boost_geometry_util::Point2D(0, 0), boost_geometry_util::Point2D(3, 3));
geometry_msgs::msg::Point ros_point;
{
ros_point.x = 2.0;
ros_point.y = 2.0;
ros_point.z = 0.3;
};
boost::geometry::disjoint(b0, ros_point);
```
