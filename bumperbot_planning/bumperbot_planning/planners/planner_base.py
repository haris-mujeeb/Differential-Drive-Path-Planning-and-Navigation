#!/usr/bin/env python3
from typing import Optional
import numpy as np

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
from graph_node import GraphNode

class PlannerBase:
    def __init__(self, node):
        self.node = node
        self.map: Optional[OccupancyGrid] = None

    def set_map(self, map_msg: OccupancyGrid):
        self.map = map_msg

    def grid_to_world(self, node: GraphNode) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map.header.frame_id
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = (node.x + 0.5) * self.map.info.resolution + self.map.info.origin.position.x
        pose.pose.position.y = (node.y + 0.5) * self.map.info.resolution + self.map.info.origin.position.y
        return pose

    def world_to_grid(self, pose: Pose) -> GraphNode:
        grid_x = int((pose.position.x - self.map.info.origin.position.x) / self.map.info.resolution)
        grid_y = int((pose.position.y - self.map.info.origin.position.y) / self.map.info.resolution)
        return GraphNode(grid_x, grid_y)

    def in_bound(self, node: GraphNode):
        return 0 <= node.x < self.map.info.width and 0 <= node.y < self.map.info.height

    def is_free(self, node: GraphNode):
        idx = node.x + node.y * self.map.info.width
        return 0 <= self.map.data[idx] < 99

    def plan(self, start_pose: Pose, goal_pose: Pose) -> Optional[Path]:
        raise NotImplementedError
