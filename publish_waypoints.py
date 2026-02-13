import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
import time
import math

class WaypointPublisherNode(Node):
    def __init__(self):
        super().__init__('waypoint_publisher_node')
        self.publisher_ = self.create_publisher(PoseStamped, '/next_goal_pose', 10)
        self.get_logger().info('Waypoint Publisher Node has been started.')

        # # Define the main "Key" waypoints
        # self.key_waypoints = [
        #     (0.0, 0.0, 0.0),    # Start
        #     (0.5, 7.0, 0.0),    # Long distance target
        #     (-4.5, -8.0, 0.0),  # Another long distance target
        #     (0.0, 0.0, 0.0)     # Return home
        # ]

        # Generate the dense list of waypoints automatically
        # self.dense_waypoints = self.expand_path(self.key_waypoints, step_size=1.0)
        
        # Define the main "Key" waypoints
        self.key_waypoints = [ ((-3.0 if i % 2 == 0 else -1.0), -2.0*i, 0.0) for i in range(3) ]
        self.key_waypoints += list(reversed(self.key_waypoints[:-1]))
        
        # self.key_waypoints = [
        #     (-1.0, 0.0, 0.0),    # Start
        #     (-3.0, -2.0, 0.0),    # Start
        #     (-3.0, -2.0, 0.0),    # Start
        #     (-3.0, -2.0, 0.0),    # Start
            # (0.0, 6.0, 0.0),    # Long distance target
            # (4.5, 2.0, 0.0),  # Another long distance target
            # (0.0, 0.0, 0.0)     # Return home
        # ]
        self.dense_waypoints = self.key_waypoints

    def expand_path(self, waypoints, step_size):
        """
        Interpolates between key waypoints to create smaller segments.
        """
        if not waypoints:
            return []

        expanded_path = []
        # Start from the first point
        expanded_path.append(waypoints[0])

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i+1]
            
            # Calculate Euclidean distance
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dist = math.hypot(dx, dy)

            # Calculate number of steps needed
            # We use ceil to ensure the last step is not too long
            if dist > 0:
                num_steps = int(math.ceil(dist / step_size))
            else:
                num_steps = 1
            
            # Generate intermediate points using Linear Interpolation
            for j in range(1, num_steps + 1):
                fraction = j / num_steps
                new_x = start[0] + dx * fraction
                new_y = start[1] + dy * fraction
                
                # We interpolate Yaw simply, or just take the target Yaw
                # Here we just take the target Yaw for simplicity
                new_yaw = end[2] 
                
                expanded_path.append((new_x, new_y, new_yaw))

        return expanded_path

    def publish_waypoints(self):
        while True:
            for i, (x, y, yaw) in enumerate(self.dense_waypoints):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0

                q = quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]

                self.get_logger().info(f'Waypoint {i+1}/{len(self.dense_waypoints)}: x={x:.2f}, y={y:.2f}')

                # # Publish the same goal for 5 seconds (reduced time for intermediate points)
                # for _ in range(5):
                #     pose.header.stamp = self.get_clock().now().to_msg()
                #     self.publisher_.publish(pose)
                #     time.sleep(5)

                # Publish the same goal for 5 seconds (reduced time for intermediate points)
                pose.header.stamp = self.get_clock().now().to_msg()
                self.publisher_.publish(pose)
                time.sleep(10)

            self.get_logger().info('Path completed.')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisherNode()
    node.publish_waypoints()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()