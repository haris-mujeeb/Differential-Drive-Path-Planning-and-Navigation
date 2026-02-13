#!/usr/bin/env python3
from typing import Optional
import heapq
import numpy as np
from numba import njit, types
from numba.typed import Dict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import rclpy.duration
from graph_node import GraphNode

@njit
def a_star(start_x, start_y, goal_x, goal_y, data, width, height):
    q = [(0.0, (start_x, start_y))]
    
    costs = Dict.empty(key_type=types.intp, value_type=types.float64)
    start_idx = start_x + start_y * width
    costs[start_idx] = 0.0
    
    prev_nodes = Dict.empty(key_type=types.intp, value_type=types.intp)

    dirs = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,-1), (1,-1), (-1,1)]
    
    while q:
        f_cost, current_coords = heapq.heappop(q)
        current_x, current_y = current_coords
        current_idx = current_x + current_y * width
        
        g_cost = costs.get(current_idx, np.inf)

        if current_x == goal_x and current_y == goal_y:
            return prev_nodes, costs

        if f_cost > g_cost + np.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2):
            continue

        for dx, dy in dirs:
            nx, ny = current_x + dx, current_y + dy
            
            if not (0 <= nx < width and 0 <= ny < height):
                continue
            
            n_idx = nx + ny * width
            if not(0 <= data[n_idx] < 99):
                continue
            
            move_cost = 1.0 if dx == 0 or dy == 0 else np.sqrt(2)
            new_g_cost = g_cost + move_cost + data[n_idx]
            
            if new_g_cost < costs.get(n_idx, np.inf):
                costs[n_idx] = new_g_cost
                prev_nodes[n_idx] = current_idx
                heuristic = np.sqrt((goal_x - nx)**2 + (goal_y - ny)**2)
                new_f_cost = new_g_cost + heuristic
                heapq.heappush(q, (new_f_cost, (nx, ny)))
    
    return prev_nodes, costs

class AStarPlanner(Node):
  def __init__(self):
    super().__init__("a_star_node")
    self.tf_buffer_ = Buffer()
    self.tf_listener_ = TransformListener(self.tf_buffer_, self)

    map_qos_ = QoSProfile(depth=10)
    map_qos_.durability = DurabilityPolicy.TRANSIENT_LOCAL
    map_qos_.reliability = ReliabilityPolicy.RELIABLE
    
    self.map_sub_ = self.create_subscription(OccupancyGrid, "/costmap", self.map_callback, map_qos_)
    self.goal_pose_sub_ = self.create_subscription(PoseStamped, "/goal_pose", self.goal_pose_callback, 10)

    self.path_pub_ = self.create_publisher(Path, "/a_star/path", 10)
    self.visited_map_pub_ = self.create_publisher(OccupancyGrid, "/a_star/visited_map", 10)

    self.map_ : Optional[OccupancyGrid] = None
  
  def map_callback(self, map_msg : OccupancyGrid):
    self.map_ = map_msg
    if self.map_ is None:
      self.get_logger().error("No map received!") 

  def goal_pose_callback(self, pose_msg : PoseStamped):
    if self.map_ is None:
      self.get_logger().error("No map received!") 
      return
    
    try:
      map_to_base_tf = self.tf_buffer_.lookup_transform(
          self.map_.header.frame_id, 
          "base_footprint", 
          rclpy.time.Time(), 
          timeout=rclpy.duration.Duration(seconds=0.1)
      )
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
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
        self.get_logger().warning("No map found")
        return None
    
    start_node = self.world_to_grid(start_pose)
    goal_node = self.world_to_grid(goal_pose)

    if not self.in_bound(start_node) or not self.in_bound(goal_node) or \
       not self.is_free(start_node) or not self.is_free(goal_node):
        self.get_logger().warning("Start or goal node is invalid")
        return None
    
    prev_nodes, costs = a_star(start_node.x, start_node.y, goal_node.x, goal_node.y, 
                               np.array(self.map_.data), self.map_.info.width, self.map_.info.height)

    goal_idx = goal_node.x + goal_node.y * self.map_.info.width
    if goal_idx not in prev_nodes:
        self.get_logger().warning("No path found to the goal")
        return None

    # Visualize visited nodes
    visited_map = OccupancyGrid()
    visited_map.header.frame_id = self.map_.header.frame_id
    visited_map.info = self.map_.info
    visited_map.data = np.full(self.map_.info.width * self.map_.info.height, -1, dtype=np.int8).tolist()
    for idx in costs.keys():
        visited_map.data[idx] = 100
    self.visited_map_pub_.publish(visited_map)

    # Reconstruct path
    path = Path()
    path.header.stamp = self.get_clock().now().to_msg()
    path.header.frame_id = self.map_.header.frame_id
    
    path_nodes = []
    start_idx = start_node.x + start_node.y * self.map_.info.width
    current_idx = goal_idx
    while current_idx != start_idx:
        x = current_idx % self.map_.info.width
        y = current_idx // self.map_.info.width
        path_nodes.append(GraphNode(x, y))
        current_idx = prev_nodes.get(current_idx)
        if current_idx is None:
            self.get_logger().error("Path reconstruction failed")
            return None
    path_nodes.append(start_node)
    path_nodes.reverse()

    path.poses = [self.grid_to_world(node) for node in path_nodes]
    return path

  def grid_to_world(self, node: GraphNode) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = self.map_.header.frame_id
    pose.header.stamp = self.get_clock().now().to_msg()
    pose.pose.position.x = (node.x + 0.5) * self.map_.info.resolution + self.map_.info.origin.position.x
    pose.pose.position.y = (node.y + 0.5) * self.map_.info.resolution + self.map_.info.origin.position.y
    return pose

  def world_to_grid(self, pose : Pose) -> GraphNode:
    grid_x = int((pose.position.x - self.map_.info.origin.position.x) / self.map_.info.resolution)
    grid_y = int((pose.position.y - self.map_.info.origin.position.y) / self.map_.info.resolution)
    return GraphNode(grid_x, grid_y)
  
  def in_bound(self, node : GraphNode):
    return 0 <= node.x < self.map_.info.width and 0 <= node.y < self.map_.info.height

  def is_free(self, node : GraphNode):
    idx = node.x + node.y * self.map_.info.width
    return 0 <= self.map_.data[idx] < 99


def main(args=None):
  rclpy.init(args=args)
  # Allow Numba to warm up
  a_star(0,0,1,1,np.array([0,0,0,0]),2,2)
  node = AStarPlanner()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == '__main__':
  main()
