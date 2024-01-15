#ifndef BEZIER_PATH_PLANNING__BEZIER_MATH_HPP_
#define BEZIER_PATH_PLANNING__BEZIER_MATH_HPP_

#include "geometry_msgs/msg/point.hpp"
#include "turtlesim/msg/pose.hpp"

namespace bezier_path_planning
{
class BezierMath
{
public:
  static geometry_msgs::msg::Point calculateBezierPoint(float t, const turtlesim::msg::Pose& start, const geometry_msgs::msg::Point& control1, const geometry_msgs::msg::Point& control2, const turtlesim::msg::Pose& end);
};
}  // namespace bezier_path_planning

#endif  // BEZIER_PATH_PLANNING__BEZIER_MATH_HPP_