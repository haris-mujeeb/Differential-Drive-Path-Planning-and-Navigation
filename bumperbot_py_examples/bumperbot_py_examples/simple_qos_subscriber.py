import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class SimpleQoSSuscriber(Node):

  def __init__(self):
    super().__init__("simple_qos_subscriber")
    self.qos_profile_sub_ = QoSProfile(depth=10)
    self.counter = 0

    self.declare_parameter('reliability', 'system_default')
    self.declare_parameter('durability', 'system_default')

    reliability = self.get_parameter('reliability').get_parameter_value().string_value  
    durability = self.get_parameter('durability').get_parameter_value().string_value
    
    if reliability == 'best_effort':
      self.qos_profile_sub_.reliability = QoSReliabilityPolicy.BEST_EFFORT
      self.get_logger().info('[Reliability] : Best Effort')
    elif reliability == 'reliable':
      self.qos_profile_sub_.reliability = QoSReliabilityPolicy.RELIABLE
      self.get_logger().info('[Reliability] : Realiable')
    elif reliability == 'system_default':
      self.qos_profile_sub_.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
      self.get_logger().info('[Reliability] : System Default')
    else:
      self.get_logger().error("Selected Qos: %s doesn\'t exist!" % reliability)
      return

    if durability == 'volatile':
      self.qos_profile_sub_.durability = QoSDurabilityPolicy.VOLATILE
      self.get_logger().info('[Durability] : Volatile')
    elif durability == 'transient_local':
      self.qos_profile_sub_.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
      self.get_logger().info('[Durability] : Transient Local')
    elif durability == 'system_default':
      self.qos_profile_sub_.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
      self.get_logger().info('[Durability] : System Default')
    else:
      self.get_logger().error('Selected Durability QoS: %s doesn\'t exist!' % durability)
      return

    self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, qos_profile=self.qos_profile_sub_)

  def msgCallback(self, msg : String):
    self.get_logger().info("I heard: %s" % msg.data)


def main(args=None):
  rclpy.init(args=args)

  simple_qos_subscriber = SimpleQoSSuscriber()
  rclpy.spin(simple_qos_subscriber)
  
  simple_qos_subscriber.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()