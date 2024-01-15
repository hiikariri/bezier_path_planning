#ifndef BEZIER_PATH_PLANNING__BEZIER_HPP_
#define BEZIER_PATH_PLANNING__BEZIER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "bezier_path_planning/utils/bezier_math.hpp"

namespace bezier_path_planning
{
class Bezier : public rclcpp::Node
{
public:
  Bezier();
  ~Bezier();
  void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);
  std::string twistToString(const geometry_msgs::msg::Twist& twist);
  std::string pointToString(const geometry_msgs::msg::Point& point);
  void teleportTurtle(float x, float y);
  geometry_msgs::msg::Point calculatePoint(float t);
  void moveAlongBezierCurve();
private:
  turtlesim::msg::Pose start_pose_;
  turtlesim::msg::Pose end_pose_;
  geometry_msgs::msg::Point control_point1_;
  geometry_msgs::msg::Point control_point2_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
  turtlesim::msg::Pose turtle_pose_;
  rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
};
}  // namespace bezier_path_planning

#endif  // BEZIER_PATH_PLANNING__BEZIER_HPP_