#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose, PoseStamped
from bumperbot_msgs.action import GetPath
from bumperbot_planning.planners.a_star import AStarPlanner
from bumperbot_planning.planners.dijkstra import DijkstraPlanner
import rclpy.duration

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__("global_planner")

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)
        
        map_qos_ = QoSProfile(depth=10)
        map_qos_.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos_.reliability = ReliabilityPolicy.RELIABLE
        
        self.map_sub_ = self.create_subscription(OccupancyGrid, "/costmap", self.map_callback, map_qos_)
        
        self.path_pub_ = self.create_publisher(Path, "/global_path", 10)
        
        self.action_server = ActionServer(self, GetPath, 'get_path', self.get_path_callback)
        
        self.map = None
        self.planners = {
            'a_star': AStarPlanner(self),
            'dijkstra': DijkstraPlanner(self),
        }

    def map_callback(self, map_msg: OccupancyGrid):
        self.map = map_msg
        for planner in self.planners.values():
            planner.set_map(self.map)

    def get_path_callback(self, goal_handle):
        goal = goal_handle.request
        planner_name = goal.planner
        
        if self.map is None:
            goal_handle.abort()
            self.get_logger().error("No map received!")
            return GetPath.Result()

        if planner_name not in self.planners:
            goal_handle.abort()
            self.get_logger().error(f"Planner {planner_name} not found!")
            return GetPath.Result()

        try:
            map_to_base_tf = self.tf_buffer_.lookup_transform(
                self.map.header.frame_id, 
                "base_footprint", 
                rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            goal_handle.abort()
            self.get_logger().error(f"Could not transform from map to base_footprint: {e}")
            return GetPath.Result()
        
        start_pose = Pose()
        start_pose.position.x = map_to_base_tf.transform.translation.x
        start_pose.position.y = map_to_base_tf.transform.translation.y
        start_pose.orientation = map_to_base_tf.transform.rotation

        planner = self.planners[planner_name]
        path = planner.plan(start_pose, goal.goal_pose.pose)

        if path is None:
            goal_handle.abort()
            self.get_logger().warning("No path found")
            return GetPath.Result()

        if path.poses:
            self.get_logger().info("Shortest path found!")
            self.path_pub_.publish(path)
            goal_handle.succeed()
            result = GetPath.Result()
            result.path = path
            return result
        else:
            goal_handle.abort()
            self.get_logger().warning("Path found but with no poses")
            return GetPath.Result()

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
