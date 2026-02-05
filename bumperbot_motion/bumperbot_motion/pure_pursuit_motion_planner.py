#!/usr/bin/env python3
import math
import rclpy
from typing import List
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Twist
from tf2_ros import Buffer, TransformListener, TransformStamped
from tf_transformations import quaternion_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, inverse_matrix


class PPMotionPlanner(Node):
  def __init__(self):
    super().__init__("pure_pursuit_motion_planner_node")
    
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.declare_parameter("look_ahead_distance", 0.4)
    self.declare_parameter("goal_tolerance", 0.1)
    self.declare_parameter("max_linear_velocity", 0.3)
    self.declare_parameter("max_angular_velocity", 1.0)
    
    self.look_ahead_distance = self.get_parameter("look_ahead_distance").value
    self.goal_tolerance = self.get_parameter("goal_tolerance").value
    self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
    self.max_angular_velocity = self.get_parameter("max_angular_velocity").value

    self.global_path : Path = None
    self.path_sub = self.create_subscription(Path, "/a_star/path", self.path_callback_, 10)
    self.path_pub = self.create_timer(0.1, self.control_loop)
    
    self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10) 
    self.next_pose_pub = self.create_publisher(PoseStamped, "/motion_planner/next_pose", 10) 


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
    next_pose_robot = PoseStamped()
    next_pose_robot.pose.position.x = next_pose_robot_tf[0][3]
    next_pose_robot.pose.position.y = next_pose_robot_tf[1][3]
    next_pose_robot.pose.position.z = next_pose_robot_tf[2][3]
    quaternion = quaternion_from_matrix(next_pose_robot_tf)
    next_pose_robot.pose.orientation.x = quaternion[0]
    next_pose_robot.pose.orientation.y = quaternion[1]
    next_pose_robot.pose.orientation.z = quaternion[2]
    next_pose_robot.pose.orientation.w = quaternion[3]
    curvature = self.get_curvature_(next_pose_robot.pose)
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = self.max_linear_velocity
    cmd_vel_msg.angular.z = curvature * self.max_angular_velocity
    self.cmd_vel_pub.publish(cmd_vel_msg)


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
      if distance > self.look_ahead_distance:
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

  def get_curvature_(self, carrot_pose : Pose):
    L_squared = carrot_pose.position.x **2 + carrot_pose.position.y ** 2
    if L_squared > 0.001:
      return 2.0 * carrot_pose.position.y / L_squared
    else: 
      return 0.0


def main(args=None):
  rclpy.init(args=args)
  pd_motion_planner = PPMotionPlanner()
  rclpy.spin(pd_motion_planner)
  rclpy.shutdown(pd_motion_planner)

if __name__ == "__main__":
  main()