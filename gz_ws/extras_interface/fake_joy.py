#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

class JoyFakePublisher(Node):
    def __init__(self):
        super().__init__('joy_fake_publisher')
        # Publisher to /joy_fake topic with Joy message type
        self.joy_pub = self.create_publisher(Joy, '/joy_fake', 10)
        # Subscriber to /diff_drive topic with Vector3 message type
        self.create_subscription(Vector3, '/diff_drive', self.diff_drive_callback, 10)
        
        # Create a timer that publishes at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Initialize a Joy message to mimic Logitech F310
        self.joy_msg = Joy()
        # For example, assume the F310 has 6 axes and 10 buttons (adjust as needed)
        self.joy_msg.axes = [0.0] * 6
        self.joy_msg.buttons = [0] * 10

        # Variables for handling diff_drive timeout
        self.last_diff_drive_time = None  # Last time a diff_drive message was received
        self.diff_drive_timeout = 1.0     # Timeout in seconds

    def diff_drive_callback(self, msg: Vector3):
        """
        Callback for /diff_drive messages.
        Copies diff_drive.x to axis 4 and diff_drive.y to axis 0,
        and records the time of the last received message.
        """
        # Update the Joy message with new values
        self.joy_msg.axes[4] = msg.x  # axis 4 gets diff_drive.x
        self.joy_msg.axes[0] = msg.y  # axis 0 gets diff_drive.y
        # Record the time of this message
        self.last_diff_drive_time = self.get_clock().now()
        self.get_logger().info(
            f"Received /diff_drive: x={msg.x}, y={msg.y} -> Updated axes[4]={msg.x}, axes[0]={msg.y}"
        )

    def timer_callback(self):
        """
        Timer callback to publish the Joy message regularly.
        It checks if the diff_drive values have timed out and resets them if needed.
        """
        current_time = self.get_clock().now()
        # Check if a diff_drive message was received recently
        if self.last_diff_drive_time is None:
            # No diff_drive message has ever been received; ensure values are at zero.
            if self.joy_msg.axes[4] != 0.0 or self.joy_msg.axes[0] != 0.0:
                self.joy_msg.axes[4] = 0.0
                self.joy_msg.axes[0] = 0.0
                self.get_logger().info("No /diff_drive messages received yet; resetting axes to 0.0.")
        else:
            # Calculate the time difference in seconds
            time_diff = (current_time - self.last_diff_drive_time).nanoseconds * 1e-9
            if time_diff > self.diff_drive_timeout:
                # If timeout exceeded, reset the axes if they are not already zero
                if self.joy_msg.axes[4] != 0.0 or self.joy_msg.axes[0] != 0.0:
                    self.joy_msg.axes[4] = 0.0
                    self.joy_msg.axes[0] = 0.0
                    self.get_logger().info("Timeout: Reset diff_drive axes to 0.0.")

        # Update the header timestamp and publish the Joy message
        self.joy_msg.header.stamp = current_time.to_msg()
        self.joy_pub.publish(self.joy_msg)
        self.get_logger().debug("Published /joy_fake message.")

def main(args=None):
    rclpy.init(args=args)
    node = JoyFakePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
