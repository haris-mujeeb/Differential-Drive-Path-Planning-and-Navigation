#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryDrawer(Node):
  def __init__(self):
    super().__init__(node_name="trajectory_drawer")
    self.odom_sub_ = self.create_subscription(Odometry,  "bumperbot_controller/odom", self.odom_callback ,10)
    self.trajectory_pub_ = self.create_publisher(Path, "bumperbot_controller/trajectory", 10)
    self.path_msg_ = Path()

  def odom_callback(self, msg: Odometry):
    self.path_msg_.header.frame_id = msg.header.frame_id

    new_pose = PoseStamped()
    new_pose.header.frame_id = msg.header.frame_id
    new_pose.header.stamp = msg.header.stamp
    new_pose.pose = msg.pose.pose

    self.path_msg_.poses.append(new_pose)
    self.trajectory_pub_.publish(self.path_msg_)


def main(args=None):
  rclpy.init(args=args)
  trajectory_drawer = TrajectoryDrawer()
  rclpy.spin(trajectory_drawer)
  rclpy.shutdown()

if __name__ == "__main__":
  main()