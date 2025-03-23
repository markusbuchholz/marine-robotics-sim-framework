#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import threading
import time
import os

class ServoOutputsLogger(Node):
    def __init__(self):
        super().__init__('servo_outputs_logger')

        # Declare and get parameters
        self.declare_parameter('sample_rate', 100)  # Samples per second
        self.declare_parameter('output_file', 'servo_outputs_log.txt')  # Output file path

        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        if self.sample_rate <= 0:
            self.get_logger().warn(f'Invalid sample_rate {self.sample_rate}. Setting to 100 Hz.')
            self.sample_rate = 100

        # Initialize variables
        self.lock = threading.Lock()
        self.latest_pwm = [0] * 8  # Assuming 8 thrusters
        self.start_time = None
        self.elapsed_time = 0.0

        # Open the output file
        try:
            # Ensure the directory exists
            output_dir = os.path.dirname(self.output_file)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir)
            self.file = open(self.output_file, 'w')
            self.get_logger().info(f'Logging servo outputs to {self.output_file}')
            # Write header
            header = 'Time(s)\tServo1\tServo2\tServo3\tServo4\tServo5\tServo6\tServo7\tServo8\n'
            self.file.write(header)
        except Exception as e:
            self.get_logger().error(f'Failed to open file {self.output_file}: {e}')
            raise

        # Subscribe to /bluerov2/servo_outputs
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/bluerov2/servo_outputs',
            self.servo_outputs_callback,
            10  # QoS history depth
        )
        self.subscription  # prevent unused variable warning

        # Create a timer for logging
        timer_period = 1.0 / self.sample_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'Servo Outputs Logger initialized with sample rate: {self.sample_rate} Hz')

    def servo_outputs_callback(self, msg):
        with self.lock:
            # Ensure we have at least 8 servo outputs
            if len(msg.data) < 8:
                self.get_logger().warn(f'Received servo_outputs with insufficient data: {len(msg.data)} elements.')
                # Pad with zeros if necessary
                self.latest_pwm = msg.data[:8] + [0]*(8 - len(msg.data))
            else:
                self.latest_pwm = msg.data[:8]

    def timer_callback(self):
        with self.lock:
            current_time = self.get_clock().now()
            if self.start_time is None:
                # Initialize start_time on first callback
                self.start_time = current_time
                self.elapsed_time = 0.0
            else:
                # Calculate elapsed time in seconds
                elapsed_duration = current_time - self.start_time
                # Ensure the duration is valid
                if elapsed_duration.nanoseconds < 0:
                    self.get_logger().warn('Elapsed duration is negative. Resetting start_time.')
                    self.start_time = current_time
                    self.elapsed_time = 0.0
                else:
                    self.elapsed_time = elapsed_duration.nanoseconds * 1e-9

            # Prepare the line to write
            line = f"{self.elapsed_time:.6f}\t" + "\t".join(map(str, self.latest_pwm)) + "\n"

            # Write to file
            try:
                self.file.write(line)
                self.file.flush()  # Ensure data is written to disk
            except Exception as e:
                self.get_logger().error(f'Failed to write to file: {e}')

    def shutdown(self):
        self.get_logger().info('Shutting down Servo Outputs Logger...')
        try:
            if hasattr(self, 'file') and not self.file.closed:
                self.file.close()
                self.get_logger().info(f'Closed file {self.output_file}')
        except Exception as e:
            self.get_logger().error(f'Error closing file: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ServoOutputsLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected. Initiating shutdown...')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
