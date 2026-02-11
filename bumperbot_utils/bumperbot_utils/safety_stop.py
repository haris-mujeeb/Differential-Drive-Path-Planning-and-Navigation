#!/usr/bin/env python3
import time
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist # Added Twist import
from visualization_msgs.msg import Marker, MarkerArray


class State(Enum):
  FREE = 0
  WARNING = 1
  DANGER = 2


class SafetyStop(Node):
  def __init__(self):
    super().__init__('safety_stop_node')
    self.declare_parameter('warning_distance', 0.6)
    self.declare_parameter('danger_distance', 0.2)
    self.declare_parameter('scan_topic', 'scan')
    self.declare_parameter('safety_stop_topic', 'safety_stop')
    
    self.warning_distance = self.get_parameter('warning_distance').get_parameter_value().double_value
    self.danger_distance = self.get_parameter('danger_distance').get_parameter_value().double_value
    self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
    self.safety_stop_topic = self.get_parameter('safety_stop_topic').get_parameter_value().string_value
    
    self.is_first_msg = True
    self.state = State.FREE
    self.prev_state = State.FREE

    self.laser_sub = self.create_subscription(
        LaserScan, self.scan_topic, self.laser_callback, 10
    )
    self.cmd_vel_sub = self.create_subscription( # Subscribe to twist_mux output
        Twist, "bumperbot_controller/cmd_vel_unstamped", self.cmd_vel_callback, 10
    )
    self.safety_stop_pub = self.create_publisher(
        Bool, self.safety_stop_topic, 10
    )
    self.final_cmd_vel_pub = self.create_publisher( # Publish final filtered commands
        Twist, "/final_cmd_vel", 10
    )
    self.zones_pub = self.create_publisher(
        MarkerArray, 'zones', 10
    )
    # JoyTurbo action clients are removed
    # self.decrease_speed_client = ActionClient(self, JoyTurbo, 'joy_turbo_decrease')
    # self.increase_speed_client = ActionClient(self, JoyTurbo, 'joy_turbo_increase')
    # while not self.decrease_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
    #   self.get_logger().warn('Action /joy_turbo_decrease not available! Waiting..')
    #   time.sleep(2.0)

    # while not self.increase_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
    #   self.get_logger().warn('Action /joy_turbo_increase not available! Waiting..')
    #   time.sleep(2.0)
    
    # Prepare Zones for Visualization
    self.zones = MarkerArray()
    warning_zone = Marker()
    warning_zone.id = 0
    warning_zone.type = Marker.CYLINDER
    warning_zone.action = Marker.ADD
    warning_zone.scale.z = 0.001
    warning_zone.scale.x = self.warning_distance * 2
    warning_zone.scale.y = self.warning_distance * 2
    warning_zone.color.r = 1.0
    warning_zone.color.g = 0.984
    warning_zone.color.b = 0.0
    warning_zone.color.a = 0.5
    danger_zone = Marker()
    danger_zone.id = 1
    danger_zone.type = Marker.CYLINDER
    danger_zone.action = Marker.ADD
    danger_zone.scale.z = 0.001
    danger_zone.scale.x = self.danger_distance * 2
    danger_zone.scale.y = self.danger_distance * 2
    danger_zone.color.r = 1.0
    danger_zone.color.g = 0.0
    danger_zone.color.b = 0.0
    danger_zone.color.a = 0.5
    danger_zone.pose.position.z = 0.01
    
    self.zones.markers = [warning_zone, danger_zone]


  def laser_callback(self, msg: LaserScan):
    self.state = State.FREE

    for range_value in msg.ranges:
      if not math.isinf(range_value) and range_value <= self.warning_distance:
        self.state = State.WARNING
        if range_value <= self.danger_distance:
          self.state = State.DANGER
          # Stop immediately!
          break

    if self.state != self.prev_state:
      if self.state == State.WARNING:
        self.zones.markers[0].color.a = 1.0
        self.zones.markers[1].color.a = 0.5
      elif self.state == State.DANGER:
        self.zones.markers[0].color.a = 1.0
        self.zones.markers[1].color.a = 1.0
      elif self.state == State.FREE:
        self.zones.markers[0].color.a = 0.5
        self.zones.markers[1].color.a = 0.5
      self.prev_state = self.state
      
    if self.is_first_msg:
      for zone in self.zones.markers:
        zone.header.frame_id = msg.header.frame_id
      self.is_first_msg = False
    self.zones_pub.publish(self.zones)

    # Publish the safety stop lock state continuously based on current state
    is_safety_stop_msg = Bool()
    if self.state == State.DANGER:
        is_safety_stop_msg.data = True
    else:
        is_safety_stop_msg.data = False
    self.safety_stop_pub.publish(is_safety_stop_msg)


  def cmd_vel_callback(self, msg: Twist):
    clamped_twist = Twist()
    clamped_twist.linear.x = msg.linear.x
    clamped_twist.angular.z = msg.angular.z

    if self.state == State.DANGER:
      clamped_twist.linear.x = 0.0
      clamped_twist.angular.z = 0.0
    elif self.state == State.WARNING:
      # Clamp linear velocity if it exceeds the predefined limit
      max_warning_linear_velocity = 0.2
      max_warning_angular_velocity = 0.5
      if abs(msg.linear.x) > max_warning_linear_velocity:
          clamped_twist.linear.x = max_warning_linear_velocity * (1 if msg.linear.x > 0 else -1)
      # Clamp angular velocity if it exceeds the predefined limit
      if abs(msg.angular.z) > max_warning_angular_velocity:
          clamped_twist.angular.z = max_warning_angular_velocity * (1 if msg.angular.z > 0 else -1)
    elif self.state == State.FREE:
      # Pass through original message, no clamping
      pass # Already copied msg to clamped_twist
    
    self.final_cmd_vel_pub.publish(clamped_twist)


def main(args=None):
  rclpy.init(args=args)
  safety_stop = SafetyStop()
  rclpy.spin(safety_stop)
  rclpy.shutdown()

if __name__ == "__main__":
  main()