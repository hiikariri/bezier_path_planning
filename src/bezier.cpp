#include <cstdlib>
#include <ctime>
#include <sstream>

#include "bezier_path_planning/bezier.hpp"

namespace bezier_path_planning
{
  Bezier::Bezier() : Node("bezier_path_planning")
  {
    std::srand(std::time(nullptr)); // seed the random number generator
  
    start_pose_.x = 0;
    start_pose_.y = 0;
    start_pose_.theta = 0;

    end_pose_.x = 10;  // Use fixed values for testing
    end_pose_.y = 10;
    end_pose_.theta = 0;

    control_point1_.x = 3;
    control_point1_.y = 5;

    control_point2_.x = 7;
    control_point2_.y = 5;

    // publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

    // subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
    // "/turtle1/pose", 10, std::bind(&Bezier::poseCallback, this, std::placeholders::_1));
    
    teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
  }

  Bezier::~Bezier()
  {

  }

  void Bezier::poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    turtle_pose_ = *msg;
  }
  
  std::string Bezier::twistToString(const geometry_msgs::msg::Twist& twist)
  {
    std::stringstream ss;
    ss << "linear: {x: " << twist.linear.x << ", y: " << twist.linear.y << ", z: " << twist.linear.z << "}, ";
    ss << "angular: {x: " << twist.angular.x << ", y: " << twist.angular.y << ", z: " << twist.angular.z << "}";
    return ss.str();
  }

  std::string Bezier::pointToString(const geometry_msgs::msg::Point& point)
  {
    std::stringstream ss;
    ss << "x: " << point.x << ", y: " << point.y << ", z: " << point.z;
    return ss.str();
  }

  void Bezier::teleportTurtle(float x, float y)
  {
  auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
  request->x = x;
  request->y = y;

  // Wait for the service to be available
  while (!teleport_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
  }

  // Send the request
  auto result_future = teleport_client_->async_send_request(request);
  }

  geometry_msgs::msg::Point Bezier::calculatePoint(float t)
  {
    geometry_msgs::msg::Point current_point = BezierMath::calculateBezierPoint(t, start_pose_, control_point1_, control_point2_, end_pose_);
    // geometry_msgs::msg::Point next_point = BezierMath::calculateBezierPoint(t + 0.01, start_pose_, control_point1_, control_point2_, end_pose_);
    std::cout << pointToString(current_point) << std::endl;

    // geometry_msgs::msg::Twist twist;
    // float dx = next_point.x - current_point.x;
    // float dz = next_point.z - current_point.z;
    // float distance = std::sqrt(dx * dx + dz * dz);
    // float angle = std::atan2(dz, dx);
  
    // twist.linear.x = distance * 100; // forward speed
    // twist.angular.z = angle; // turn rate
    // std::cout << twistToString(twist) << std::endl;

    // // publisher_->publish(twist);

    teleportTurtle(current_point.x, current_point.z);  // Teleport the turtle to the calculated point
  }

  void Bezier::moveAlongBezierCurve()
  {
    for (float t = 0; t <= 1; t += 0.01)
    {
      calculatePoint(t);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
}  // namespace bezier_path_planning

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bezier_path_planning::Bezier>();
  node->moveAlongBezierCurve();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

