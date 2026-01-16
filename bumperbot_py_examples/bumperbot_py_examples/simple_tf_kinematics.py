import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class SimpleTfKinematics(Node):
  def __init__(self):
    super().__init__("simple_tf_kinematics")
    self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
    self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)  
    
    self.static_transform_stamped_ = TransformStamped()
    self.dynamic_transform_stamped_ = TransformStamped()

    self.x_increment_ = 0.05
    self.last_x_ = 0.0

    self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
    self.static_transform_stamped_.header.frame_id = "bumperbot_base"
    self.static_transform_stamped_.child_frame_id = "bumperbot_top"
    self.static_transform_stamped_.transform.translation.x = 0.0
    self.static_transform_stamped_.transform.translation.y = 0.0
    self.static_transform_stamped_.transform.translation.z = 0.3
    self.static_transform_stamped_.transform.rotation.x = 0.0
    self.static_transform_stamped_.transform.rotation.y = 0.0
    self.static_transform_stamped_.transform.rotation.z = 0.0
    self.static_transform_stamped_.transform.rotation.w = 1.0

    self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)

    self.get_logger().info("Publishing static transform between %s and %s " %
                           (self.static_transform_stamped_.header.frame_id,
                            self.static_transform_stamped_.child_frame_id))
    
    self.timer_ = self.create_timer(0.1, self.timerCallback)


  def timerCallback(self):
    self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
    self.dynamic_transform_stamped_.header.frame_id = "odom"
    self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"
    self.dynamic_transform_stamped_.transform.translation.x = self.x_increment_ + self.last_x_
    self.dynamic_transform_stamped_.transform.translation.y = 0.0
    self.dynamic_transform_stamped_.transform.translation.z = 0.0
    self.dynamic_transform_stamped_.transform.rotation.x = 0.0
    self.dynamic_transform_stamped_.transform.rotation.y = 0.0
    self.dynamic_transform_stamped_.transform.rotation.z = 0.0
    self.dynamic_transform_stamped_.transform.rotation.w = 1.0

    self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)

    self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x


def main(args=None):
    rclpy.init(args=args)
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()