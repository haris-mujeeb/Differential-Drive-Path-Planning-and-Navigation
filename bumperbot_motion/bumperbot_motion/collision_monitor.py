#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from enum import Enum
from bumperbot_msgs.msg import CollisionState
from visualization_msgs.msg import Marker, MarkerArray # Added for marker visualization


class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor_node')
        self.get_logger().info("CollisionMonitor node started.")

        self.declare_parameter('warning_distance', 0.5)
        self.declare_parameter('danger_distance', 0.2)
        self.declare_parameter('scan_topic', 'scan')

        self.warning_distance = self.get_parameter('warning_distance').value
        self.danger_distance = self.get_parameter('danger_distance').value
        self.scan_topic = self.get_parameter('scan_topic').value

        self.is_first_msg = True # Added for marker initialization
        self.state = State.FREE
        self.prev_state = State.FREE # Added for marker state change detection

        self.laser_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.laser_callback,
            10
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            "/cmd_vel",  # From twist_mux_topics.yaml
            self.cmd_vel_callback,
            10
        )

        self.safe_cmd_vel_pub = self.create_publisher(
            Twist,
            "/safe_cmd_vel", # From twist_mux_topics.yaml
            10
        )

        self.zones_pub = self.create_publisher( # Added for marker visualization
            MarkerArray, 'zones', 10
        )

        self.collision_state_pub = self.create_publisher(
            CollisionState,
            "/collision_monitor/state",
            10
        )

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
        self.get_logger().debug("LaserScan received.")
        # Determine state based on ranges
        current_state = State.FREE
        min_range = float('inf')
        for r in msg.ranges:
            if not math.isinf(r) and not math.isnan(r): # Added check for nan values
                min_range = min(min_range, r)

        if min_range <= self.danger_distance:
            current_state = State.DANGER
        elif min_range <= self.warning_distance:
            current_state = State.WARNING
        
        self.state = current_state # Update the actual state

        collision_state_msg = CollisionState()
        if self.state == State.FREE:
            collision_state_msg.state = CollisionState.FREE
        elif self.state == State.WARNING:
            collision_state_msg.state = CollisionState.WARNING
        elif self.state == State.DANGER:
            collision_state_msg.state = CollisionState.DANGER
        self.collision_state_pub.publish(collision_state_msg)

        self.get_logger().debug(f"Current state: {self.state.name}")

        # Marker visualization logic
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


    def cmd_vel_callback(self, msg: Twist):
        clamped_twist = Twist()
        clamped_twist.linear.x = msg.linear.x
        clamped_twist.angular.z = msg.angular.z

        if self.state == State.DANGER:
            clamped_twist.linear.x = 0.0
            clamped_twist.angular.z = 0.5
            self.get_logger().warn("DANGER state: Stopping robot.")
        elif self.state == State.WARNING:
            max_warn_linear = 0.2
            max_warn_angular = 0.5
            clamped_twist.linear.x = max(min(msg.linear.x, max_warn_linear), -max_warn_linear)
            clamped_twist.angular.z = max(min(msg.angular.z, max_warn_angular), -max_warn_angular)
            self.get_logger().warn("WARNING state: Clamping velocity.")
        else: # FREE
            self.get_logger().debug("FREE state: Passing through velocity.")

        self.safe_cmd_vel_pub.publish(clamped_twist)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
