#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <memory>

class SimpleTfKinematics : public rclcpp::Node
{
private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  geometry_msgs::msg::TransformStamped static_transform_stamped_;
  geometry_msgs::msg::TransformStamped dynamic_transform_stamped_;

  rclcpp::TimerBase::SharedPtr timer_;

  double x_increment_, last_x_;

  void timerCallback();

public:
  SimpleTfKinematics(const std::string& name);
  
}; 