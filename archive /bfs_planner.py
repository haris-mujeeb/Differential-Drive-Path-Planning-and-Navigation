#!/usr/bin/env python3
from typing import Optional, Dict, Deque
from collections import deque
import rclpy
from rclpy.node import Node  
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, LookupException
from graph_node import GraphNode

class BFSPlanner(Node):
  def __init__(self):
    super().__init__("BFSPlanner")
    self.tf_buffer_ = Buffer()
    self.tf_listener_ = TransformListener(self.tf_buffer_, self)

    map_qos_ = QoSProfile(depth=10)
    map_qos_.durability = DurabilityPolicy.TRANSIENT_LOCAL
    map_qos_.reliability = ReliabilityPolicy.RELIABLE
    
    self.map_sub_ = self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos_)
    self.goal_pose_sub_ = self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)

    self.path_pub_ = self.create_publisher(Path, "/bfs/path", 10)
    self.visited_map_pub_ = self.create_publisher(OccupancyGrid, "/bfs/visited_map", 10)

    self.map_ : Optional[OccupancyGrid] = None
    self.visited_map_ = OccupancyGrid()
  
  def map_callback(self, map_msg : OccupancyGrid):
    self.map_ = map_msg

    if self.map_ is None:
      self.get_logger().error("No map received!") 

    self.visited_map_.header.frame_id = self.map_.header.frame_id
    self.visited_map_.info = self.map_.info
    self.visited_map_.data = [-1] * (self.map_.info.height * self.map_.info.width)


  def goal_pose_callback(self, pose_msg : PoseStamped):
    if self.map_ is None:
      self.get_logger().error("No map received!") 
      return
    
    try:
      map_to_base_tf = self.tf_buffer_.lookup_transform(
          self.map_.header.frame_id, "base_footprint", rclpy.time.Time()
      )
    except LookupException as e:
      self.get_logger().error(f"Could not transform from map to base_footprint: {e}")
      return
    
    map_to_base_pose = Pose()
    map_to_base_pose.position.x = map_to_base_tf.transform.translation.x
    map_to_base_pose.position.y = map_to_base_tf.transform.translation.y
    map_to_base_pose.orientation = map_to_base_tf.transform.rotation
    
    path = self.plan(map_to_base_pose, pose_msg.pose)

    if path is None:
        self.get_logger().warning(
            "No path found from %s to base_footprint" % self.map_.header.frame_id
        )
        return

    if path.poses:
        self.get_logger().info("Shortest path found!")
        self.path_pub_.publish(path)
    else:
        self.get_logger().warning(
            "Path found but with no poses"
        )
  
  def plan(self, start_pose : Pose, goal_pose : Pose) -> Optional[Path]:
    if self.map_ is None:
      self.get_logger().warning(
          "No map found at /map to base_footprint"
      )
      return None
    
    start_node = self.world_to_grid(start_pose)
    goal_node = self.world_to_grid(goal_pose)

    if not self.in_bound(start_node):
        self.get_logger().warning("Start node is out of bounds")
        return None
    if not self.in_bound(goal_node):
        self.get_logger().warning("Goal node is out of bounds")
        return None

    if not self.is_free(start_node):
        self.get_logger().warning("Start node is in an obstacle or unknown location")
        return None
    if not self.is_free(goal_node):
        self.get_logger().warning("Goal node is in an obstacle or unknown location")
        return None

    q : Deque[GraphNode] = deque()
    q.append(start_node)
    visited = { start_node }
    dirs = [(1,0), (-1,0), (0,1), (0,-1)]

    while q:
      node = q.popleft()
      if node == goal_node:
        goal_node = node
        break
      for dx, dy in dirs:
        nx, ny = node.x + dx, node.y + dy
        neighbour = GraphNode(nx, ny, prev=node)
        if not self.in_bound(neighbour): continue
        if neighbour in visited: continue
        if not self.is_free(neighbour): continue
        visited.add(neighbour)
        q.append(neighbour)

    if goal_node not in visited:
      return None

    path = Path()
    path.header.stamp = self.get_clock().now().to_msg()
    path.header.frame_id = self.map_.header.frame_id
    
    path_nodes = []
    current_node = goal_node
    while current_node is not None:
        path_nodes.append(current_node)
        current_node = current_node.prev
    path_nodes.reverse()
    path.poses = [self.grid_to_world(node) for node in path_nodes]
    return path

  def grid_to_world(self, node: GraphNode) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = self.map_.header.frame_id
    pose.header.stamp = self.get_clock().now().to_msg()
    pose.pose.position.x = (node.x * self.map_.info.resolution) + self.map_.info.origin.position.x
    pose.pose.position.y = (node.y * self.map_.info.resolution) + self.map_.info.origin.position.y
    return pose

  def world_to_grid(self, pose : Pose) -> GraphNode:
    grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
    grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
    return GraphNode(grid_x, grid_y)
  
  def in_bound(self, node : GraphNode):
    return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

  def idx (self, node : GraphNode) -> int:
    return node.x + (node.y * self.map_.info.width)

  def is_free(self, node : GraphNode):
    return 0 <= self.map_.data[self.idx(node)] < 99


def main(args=None):
  rclpy.init(args=args)
  node = BFSPlanner()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()