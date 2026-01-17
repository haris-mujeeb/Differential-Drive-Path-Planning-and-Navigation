#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyNormalizerNode(Node):
    def __init__(self):
        super().__init__("joy_normalizer")
        self.get_logger().info("Joy normalizer node started")
        self.sub = self.create_subscription(Joy, "/joy_raw", self.joy_callback, 10)
        self.pub = self.create_publisher(Joy, "/joy", 10)

    def joy_callback(self, msg: Joy):
        new_msg = Joy()
        new_msg.header = msg.header
        new_msg.buttons = msg.buttons
        
        axes = list(msg.axes)
        if len(axes) > 3:
            # Assuming axis 3 is the one with large values
            axes[3] = axes[3] / 8.0
        
        new_msg.axes = axes
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyNormalizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
