#!/usr/bin/env python3
import math
import rclpy
from typing import List
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener, TransformStamped
from tf_transformations import quaternion_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, inverse_matrix


class PDMotionPlanner(Node):
  def __init__(self):
    super().__init__("pd_motion_planner_node")
    
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.declare_parameter("kp", 2.0)
    self.declare_parameter("kd", 0.1)
    self.declare_parameter("step_size", 0.2)
    self.declare_parameter("goal_tolerance", 0.1)
    self.declare_parameter("max_linear_velocity", 0.3)
    self.declare_parameter("max_angular_velocity", 1.0)
    
    self.kp = self.get_parameter("kp").value
    self.kd = self.get_parameter("kd").value
    self.step_size = self.get_parameter("step_size").value
    self.goal_tolerance = self.get_parameter("goal_tolerance").value
    self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
    self.max_angular_velocity = self.get_parameter("max_angular_velocity").value

    self.global_path : Path = None
    self.path_sub = self.create_subscription(Path, "/a_star/path", self.path_callback_, 10)
    self.path_pub = self.create_timer(0.1, self.control_loop)
    
    self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10) 
    self.next_pose_pub = self.create_publisher(PoseStamped, "/motion_planner/next_pose", 10) 

    self.last_cycle_time = self.get_clock().now()
    self.prev_linear_error = 0.0
    self.prev_angular_error = 0.0

  def path_callback_(self, msg : Path):
    self.global_path = msg

  def control_loop(self):
    if not self.global_path or not self.global_path.poses:
      return
    
    try:
      robot_pose_transfrom = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
    except Exception as e:
      self.get_logger().warn(f"Could not transfrom: {e}")
      return
    
    if not self.tranfrom_plan(robot_pose_transfrom.header.frame_id):
      self.get_logger().warn(f"Unable to transfrom plan in robot's frame")
      return 
    
    robot_pose = PoseStamped()
    robot_pose.header.frame_id = robot_pose_transfrom.header.frame_id
    robot_pose.pose.position.x = robot_pose_transfrom.transform.translation.x
    robot_pose.pose.position.y = robot_pose_transfrom.transform.translation.y
    robot_pose.pose.orientation = robot_pose_transfrom.transform.rotation
    
    next_pose : PoseStamped = self.get_next_pose(robot_pose)

    dx = next_pose.pose.position.x - robot_pose.pose.position.x
    dy = next_pose.pose.position.y - robot_pose.pose.position.y

    distance = math.sqrt(dx*dx + dy*dy)

    if distance < self.goal_tolerance:
      cmd = Twist()
      cmd.linear.x = 0.0
      cmd.angular.z = 0.0
      self.cmd_vel_pub.publish(cmd)
      self.get_logger().info("Goal reached!")
      return
    
    self.next_pose_pub.publish(next_pose)

    # Calculate the PDMotionPlanner command
    robot_tf = self._get_tf_matrix_from_pose(robot_pose)
    next_pose_tf = self._get_tf_matrix_from_pose(next_pose)
    
    next_pose_robot_tf = concatenate_matrices(inverse_matrix(robot_tf), next_pose_tf)
    linear_error = next_pose_robot_tf[0, 3]
    angular_error = next_pose_robot_tf[1, 3]
    
    dt = (self.get_clock().now() - self.last_cycle_time).nanoseconds * 1e-9
    if dt <= 0.0:
      return
    linear_error_derivative = (linear_error - self.prev_linear_error)/dt
    angular_error_derivative = (angular_error - self.prev_angular_error)/dt

    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = max(
      -self.max_linear_velocity,
      min(self.max_linear_velocity, self.kp * linear_error + self.kd * linear_error_derivative)
    )
    cmd_vel_msg.angular.z = max(
      -self.max_angular_velocity,
      min(self.max_angular_velocity, self.kp * angular_error + self.kd * angular_error_derivative)
    )
    self.cmd_vel_pub.publish(cmd_vel_msg)
    self.prev_linear_error = linear_error
    self.prev_angular_error = angular_error
    self.last_cycle_time = self.get_clock().now()
    

  def tranfrom_plan(self, frame : str) -> bool:
    if self.global_path.header.frame_id == frame:
      return True
    
    try:
      transfrom = self.tf_buffer.lookup_transform(frame, self.global_path.header.frame_id, rclpy.time.Time())
    except Exception as e:
      self.get_logger().warn(f"Could not transfrom plan: {e}")
      return False

    transfrom_matrix = self._get_tf_matrix_from_tf(transfrom)

    for pose in self.global_path.poses:
      pose_matrix = self._get_tf_matrix_from_pose(pose)
      transfromed_pose = concatenate_matrices(pose_matrix, transfrom_matrix)
      [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w] = quaternion_from_matrix(transfromed_pose)
      [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] = translation_from_matrix(transfromed_pose)

    self.global_path.header.frame_id = frame
    return True

  def get_next_pose(self, robot_pose : PoseStamped) -> PoseStamped:
    next_pose = self.global_path.poses[-1] # target pose is default
    
    # start from farthest and check if distance is smaller than step_size
    for pose in reversed(self.global_path.poses):
      dx = pose.pose.position.x - robot_pose.pose.position.x
      dy = pose.pose.position.y - robot_pose.pose.position.y
      distance = math.sqrt(dx*dx + dy*dy)
      if distance > self.step_size:
        next_pose = pose
      else:
        break # found the farthest point with small enough step_size
    return next_pose

  def _get_tf_matrix_from_tf(self, transfrom : TransformStamped) -> List[List[float]]:
    # ┌                ┐
    # │ R  R  R  Tx    │
    # │ R  R  R  Ty    │
    # │ R  R  R  Tz    │
    # │ 0  0  0   1    │
    # └                ┘
    transfrom_matrix = quaternion_matrix([
      transfrom.transform.rotation.x,
      transfrom.transform.rotation.y,
      transfrom.transform.rotation.z,
      transfrom.transform.rotation.w
    ])

    transfrom_matrix[0][3] = transfrom.transform.translation.x
    transfrom_matrix[1][3] = transfrom.transform.translation.y

    return transfrom_matrix
  
  def _get_tf_matrix_from_pose(self, pose : PoseStamped) -> List[List[float]]:
    # ┌                ┐
    # │ R  R  R  Tx    │
    # │ R  R  R  Ty    │
    # │ R  R  R  Tz    │
    # │ 0  0  0   1    │
    # └                ┘
    transfrom_matrix = quaternion_matrix([
      pose.pose.orientation.x,
      pose.pose.orientation.y,
      pose.pose.orientation.z,
      pose.pose.orientation.w]
    )

    transfrom_matrix[0][3] = pose.pose.position.x
    transfrom_matrix[1][3] = pose.pose.position.y

    return transfrom_matrix


def main(args=None):
  rclpy.init(args=args)
  pd_motion_planner = PDMotionPlanner()
  rclpy.spin(pd_motion_planner)
  rclpy.shutdown(pd_motion_planner)

if __name__ == "__main__":
  main()