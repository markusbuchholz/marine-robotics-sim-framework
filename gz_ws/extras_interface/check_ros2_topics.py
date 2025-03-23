#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random

#ros2 topic pub /data_1 std_msgs/msg/Float64 "{data: 1.23}"
""
class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')

        # Create two subscribers: /data_1 and /data_2.
        self.subscriber1 = self.create_subscription(
            Float64,
            '/data_1',
            self.callback_data_1,
            10  # QoS depth
        )
        self.subscriber2 = self.create_subscription(
            Float64,
            '/data_2',
            self.callback_data_2,
            10
        )

        # Create two publishers: /random_1 and /random_2.
        self.publisher1 = self.create_publisher(Float64, '/random_1', 10)
        self.publisher2 = self.create_publisher(Float64, '/random_2', 10)

        # Create a timer to publish random values every second.
        self.timer = self.create_timer(1.0, self.publish_random_values)
        self.get_logger().info("SimpleNode has been started.")

    def callback_data_1(self, msg: Float64):
        # Callback for /data_1 subscriber.
        self.get_logger().info(f"Received on /data_1: {msg.data}")

    def callback_data_2(self, msg: Float64):
        # Callback for /data_2 subscriber.
        self.get_logger().info(f"Received on /data_2: {msg.data}")

    def publish_random_values(self):
        # Generate random float values between 0 and 1.
        random_value1 = random.random()
        random_value2 = random.random()

        # Create message objects.
        msg1 = Float64()
        msg2 = Float64()
        msg1.data = random_value1
        msg2.data = random_value2

        # Publish the random values.
        self.publisher1.publish(msg1)
        self.publisher2.publish(msg2)
        self.get_logger().info(
            f"Published random values -> /random_1: {random_value1:.3f}, /random_2: {random_value2:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
