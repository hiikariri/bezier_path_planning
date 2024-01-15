#include <cmath>
#include "bezier_path_planning/utils/bezier_math.hpp"

namespace bezier_path_planning
{
geometry_msgs::msg::Point BezierMath::calculateBezierPoint(float t, const turtlesim::msg::Pose& start, const geometry_msgs::msg::Point& control1, const geometry_msgs::msg::Point& control2, const turtlesim::msg::Pose& end)
{
  geometry_msgs::msg::Point point;

  point.x = std::pow(1 - t, 3) * start.x + 3 * std::pow(1 - t, 2) * t * control1.x + 3 * (1 - t) * std::pow(t, 2) * control2.x + std::pow(t, 3) * end.x;
  point.z = std::pow(1 - t, 3) * start.y + 3 * std::pow(1 - t, 2) * t * control1.y + 3 * (1 - t) * std::pow(t, 2) * control2.y + std::pow(t, 3) * end.y;

  return point;
}
}  // namespace bezier_path_planning