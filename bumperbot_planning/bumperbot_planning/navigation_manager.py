#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from bumperbot_msgs.msg import CollisionState, GoalReached
from bumperbot_msgs.action import GetPath
from geometry_msgs.msg import PoseStamped
from collections import deque

class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')
        self.get_logger().info("Navigation Manager started.")

        self.waypoints = deque()
        self.current_goal = None
        
        self.get_path_client = ActionClient(self, GetPath, 'get_path')
        
        self.goal_reached_sub = self.create_subscription(GoalReached, '/motion_planner/goal_reached', self.goal_reached_callback, 10)
        self.collision_state_sub = self.create_subscription(CollisionState, '/collision_monitor/state', self.collision_state_callback, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.next_goal_pose_sub = self.create_subscription(PoseStamped, '/next_goal_pose', self.next_goal_pose_callback, 10)

        self.replan_timer = None

    def goal_pose_callback(self, msg: PoseStamped):
        self.waypoints.clear()
        self.waypoints.append(msg)
        self.send_next_waypoint()

    def next_goal_pose_callback(self, msg: PoseStamped):
        self.waypoints.append(msg)

    def send_next_waypoint(self):
        if self.waypoints:
            self.current_goal = self.waypoints.popleft()
            self.send_get_path_goal(self.current_goal)
        else:
            self.current_goal = None

    def goal_reached_callback(self, msg: GoalReached):
        if msg.reached:
            self.get_logger().info("Waypoint reached.")
            self.send_next_waypoint()

    def send_get_path_goal(self, pose: PoseStamped):
        goal_msg = GetPath.Goal()
        goal_msg.planner = 'a_star'
        goal_msg.goal_pose = pose
        
        self.get_path_client.wait_for_server()
        self.send_goal_future = self.get_path_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.get_path_goal_response_callback)

    def get_path_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('GetPath goal rejected')
            return
        
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_path_result_callback)

    def get_path_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Path received from global planner.')
        else:
            self.get_logger().error('GetPath action failed.')

    def collision_state_callback(self, msg: CollisionState):
        if self.current_goal and msg.state == CollisionState.WARNING:
            if self.replan_timer is None:
                self.get_logger().warn("Obstacle in warning zone, replanning...")
                self.replan_timer = self.create_timer(2.0, self.delayed_replan)

    def delayed_replan(self):
        if self.current_goal:
            self.send_get_path_goal(self.current_goal)
        self.replan_timer.destroy()
        self.replan_timer = None

def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
