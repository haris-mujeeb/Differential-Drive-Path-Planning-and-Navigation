#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import Buffer, TransformListener
from bumperbot_msgs.msg import GoalReached


class DWBMotionPlanner(Node):
    def __init__(self):
        super().__init__("dwb_motion_planner_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter("goal_tolerance", 0.2)
        self.declare_parameter("robot_radius", 0.1)

        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("min_linear_velocity", 0.0)
        self.declare_parameter("max_angular_velocity", 2.0)
        self.declare_parameter("min_angular_velocity", -2.0)
        self.declare_parameter("linear_acceleration_limit", 1.0)
        self.declare_parameter("angular_acceleration_limit", 1.0)

        self.declare_parameter("predict_time", 2.0)
        self.declare_parameter("dt", 0.1)
        self.declare_parameter("look_ahead_distance", 0.5)

        self.declare_parameter("heading_cost_gain", 2.0)
        self.declare_parameter("distance_cost_gain", 2.0)
        self.declare_parameter("velocity_cost_gain", 1.0)

        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.robot_radius = self.get_parameter("robot_radius").value

        self.max_linear_velocity = self.get_parameter("max_linear_velocity").value
        self.min_linear_velocity = self.get_parameter("min_linear_velocity").value
        self.max_angular_velocity = self.get_parameter("max_angular_velocity").value
        self.min_angular_velocity = self.get_parameter("min_angular_velocity").value
        self.linear_acceleration_limit = self.get_parameter("linear_acceleration_limit").value
        self.angular_acceleration_limit = self.get_parameter("angular_acceleration_limit").value

        self.predict_time = self.get_parameter("predict_time").value
        self.dt = self.get_parameter("dt").value
        self.look_ahead_distance = self.get_parameter("look_ahead_distance").value

        self.heading_cost_gain = self.get_parameter("heading_cost_gain").value
        self.distance_cost_gain = self.get_parameter("distance_cost_gain").value
        self.velocity_cost_gain = self.get_parameter("velocity_cost_gain").value

        self.global_path: Path = None
        self.current_odom: Odometry = None

        self.path_sub = self.create_subscription(Path, "/global_path", self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/bumperbot_controller/odom", self.odom_callback, 10)
        self.control_timer = self.create_timer(self.dt, self.control_loop)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.next_pose_pub = self.create_publisher(PoseStamped, "/motion_planner/next_pose", 10)
        self.goal_reached_pub = self.create_publisher(GoalReached, "/motion_planner/goal_reached", 10)

    def path_callback(self, msg: Path):
        self.global_path = msg

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

    def control_loop(self):
        if not self.global_path or not self.global_path.poses or not self.current_odom:
            return

        twist = self.current_odom.twist.twist
        if (math.isnan(twist.linear.x) or math.isinf(twist.linear.x) or
            math.isnan(twist.angular.z) or math.isinf(twist.angular.z)):
            self.get_logger().warn("Invalid values in odometry twist. Skipping control loop and stopping the robot.")
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            return

        try:
            robot_pose_transform = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")
            return

        robot_pose = PoseStamped()
        robot_pose.header.frame_id = robot_pose_transform.header.frame_id
        robot_pose.pose.position.x = robot_pose_transform.transform.translation.x
        robot_pose.pose.position.y = robot_pose_transform.transform.translation.y
        robot_pose.pose.orientation = robot_pose_transform.transform.rotation

        final_goal_pose = self.global_path.poses[-1]

        dx = final_goal_pose.pose.position.x - robot_pose.pose.position.x
        dy = final_goal_pose.pose.position.y - robot_pose.pose.position.y
        distance_to_final_goal = math.sqrt(dx*dx + dy*dy)

        if distance_to_final_goal < self.goal_tolerance:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info("Goal reached!")
            goal_reached_msg = GoalReached()
            goal_reached_msg.reached = True
            self.goal_reached_pub.publish(goal_reached_msg)
            self.global_path = None # Make sure we clear the path so we don't keep processing it
            return
        
        # Find the first path point that is at least look_ahead_distance away
        local_target_pose = self.global_path.poses[-1] # Default to final goal
        for pose_stamped in reversed(self.global_path.poses):
            path_point = pose_stamped.pose.position
            dist_x = path_point.x - robot_pose.pose.position.x
            dist_y = path_point.y - robot_pose.pose.position.y
            distance_from_robot = math.sqrt(dist_x**2 + dist_y**2)

            if distance_from_robot > self.look_ahead_distance:
                local_target_pose = pose_stamped
            else:
                break
        
        self.next_pose_pub.publish(local_target_pose)

        u, best_trajectory = self.dwa_control(robot_pose, local_target_pose)

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = u[0]
        cmd_vel_msg.angular.z = u[1]
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def dwa_control(self, robot_pose, goal_pose):
        dw = self.calc_dynamic_window(self.current_odom.twist.twist)
        u, best_trajectory = self.calc_control_and_trajectory(robot_pose, goal_pose, dw)
        return u, best_trajectory

    def calc_dynamic_window(self, twist):
        vs = [self.min_linear_velocity, self.max_linear_velocity,
              self.min_angular_velocity, self.max_angular_velocity]

        vd = [twist.linear.x - self.linear_acceleration_limit * self.dt,
              twist.linear.x + self.linear_acceleration_limit * self.dt,
              twist.angular.z - self.angular_acceleration_limit * self.dt,
              twist.angular.z + self.angular_acceleration_limit * self.dt]

        dw = [max(vs[0], vd[0]), min(vs[1], vd[1]),
              max(vs[2], vd[2]), min(vs[3], vd[3])]
        return dw

    def predict_trajectory(self, x_init, v, w):
        x = list(x_init)
        trajectory = [x]
        time = 0.0
        while time <= self.predict_time:
            x[2] += w * self.dt
            x[0] += v * math.cos(x[2]) * self.dt
            x[1] += v * math.sin(x[2]) * self.dt
            trajectory.append(list(x))
            time += self.dt
        return trajectory

    def calc_control_and_trajectory(self, robot_pose, goal, dw):
        x_init = [robot_pose.pose.position.x, robot_pose.pose.position.y, self.get_yaw_from_quaternion(robot_pose.pose.orientation)]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = [x_init]

        for v in [i * (dw[1] - dw[0]) / 9 + dw[0] for i in range(10)]:
            for w in [i * (dw[3] - dw[2]) / 19 + dw[2] for i in range(20)]:
                trajectory = self.predict_trajectory(x_init, v, w)

                heading_cost = self.heading_cost_gain * self.calc_heading_cost(trajectory, goal)
                dist_cost = self.distance_cost_gain * self.calc_dist_cost(trajectory)
                vel_cost = self.velocity_cost_gain * self.calc_velocity_cost(trajectory, v)
                
                final_cost = heading_cost + dist_cost + vel_cost
                
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, w]
                    best_trajectory = trajectory
        
        return best_u, best_trajectory

    def calc_heading_cost(self, trajectory, goal):
        last_x, last_y, last_yaw = trajectory[-1]
        goal_x, goal_y = goal.pose.position.x, goal.pose.position.y
        angle_to_goal = math.atan2(goal_y - last_y, goal_x - last_x)
        return abs(angle_to_goal - last_yaw)

    def calc_dist_cost(self, trajectory):
        # This cost is for obstacle avoidance, for now we will return 0
        return 0.0

    def calc_velocity_cost(self, trajectory, v):
        return self.max_linear_velocity - v

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    dwb_motion_planner = DWBMotionPlanner()
    rclpy.spin(dwb_motion_planner)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
