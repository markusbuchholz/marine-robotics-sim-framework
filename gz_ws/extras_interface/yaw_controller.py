#!/usr/bin/env python3
"""
Set Yaw Controller for Blueboat

This node subscribes to the /compass topic (which publishes the current heading
in degrees) and to the /set_yaw topic (which provides the desired yaw in degrees).
Using a PID controller, it computes the required differential thrust so that one motor
receives a negative command and the other a positive command to rotate the boat.

The node prints the current heading from the compass, and it multiplies the PID
output by a scaling factor (thrust_scale) so that a higher percentage of thrust is sent.



ros2 topic pub -1 /set_yaw std_msgs/Float32 "{data: 45.0}"

ros2 topic pub -1 /set_yaw std_msgs/Float32 "{data: 45.0}"

ros2 topic pub -1 /set_yaw std_msgs/Float32 "{data: 45.0}"

"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32
import math
import time

###############################################################################
# Simple PID Controller Class for Angular Control
###############################################################################
class PID:
    def __init__(self, kp, ki, kd, dt=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

###############################################################################
# SetYawController Node
###############################################################################
class SetYawController(Node):
    def __init__(self):
        super().__init__('set_yaw_controller')
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to /compass (current heading in degrees) and convert to radians
        self.compass_sub = self.create_subscription(
            Float32,
            '/compass',
            self.compass_callback,
            10,
            callback_group=self.callback_group)
        self.get_logger().info("Subscribed to /compass")

        # Subscribe to /set_yaw (desired yaw in degrees)
        self.set_yaw_sub = self.create_subscription(
            Float32,
            '/set_yaw',
            self.set_yaw_callback,
            10,
            callback_group=self.callback_group)
        self.get_logger().info("Subscribed to /set_yaw")

        # Publishers for thrust commands (range: -100 to 100)
        self.boat_name = "blueboat"  # Adjust if needed
        self.port_motor_pub = self.create_publisher(
            Float32,
            f'/{self.boat_name}/send_port_motor_0_100_thrust',
            10)
        self.stbd_motor_pub = self.create_publisher(
            Float32,
            f'/{self.boat_name}/send_stbd_motor_0_100_thrust',
            10)
        self.get_logger().info("Publishers for thrust commands initialized.")

        # Current measured heading (in radians) updated from /compass.
        self.current_yaw = 0.0

        # Desired yaw (in radians) set from /set_yaw. Initially, no setpoint.
        self.desired_yaw = None

        # PID controller for angular control.
        # These gains may need tuning for your vessel.
        self.angular_pid = PID(kp=2.0, ki=0.0, kd=0.5, dt=0.1)
        self.angular_pid.reset()

        # A threshold (in radians) within which we consider the boat "on target"
        self.yaw_tolerance = math.radians(2.0)  # e.g., 2° tolerance

        # New parameter: angular thrust scaling. Increase this value to amplify the effect of the PID output.
        self.declare_parameter('thrust_scale', 50.0)
        self.thrust_scale = self.get_parameter('thrust_scale').get_parameter_value().double_value

        # Timer for the control loop (e.g., 10 Hz)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def compass_callback(self, msg: Float32):
        """
        Callback for the /compass topic.
        The incoming message is assumed to be in degrees.
        We convert it to radians for internal use.
        Additionally, we print the current heading on the terminal.
        """
        # Convert the incoming heading (degrees) to radians
        self.current_yaw = math.radians(msg.data)
        # Log the current heading (in degrees)
        self.get_logger().info(f"Compass update: current heading = {msg.data:.1f}°")
        # Optionally, you could also use a print() statement:
        # print(f"Compass update: current heading = {msg.data:.1f}°")

    def set_yaw_callback(self, msg: Float32):
        """
        Callback for the /set_yaw topic.
        The incoming message is assumed to be in degrees.
        We store the desired yaw as radians.
        """
        self.desired_yaw = math.radians(msg.data)
        self.angular_pid.reset()  # reset PID state whenever a new setpoint is received
        self.get_logger().info(f"Set yaw request received: {msg.data:.1f}° (radians: {self.desired_yaw:.2f})")

    def control_loop(self):
        """
        Main control loop.
        If a desired yaw has been set, compute the heading error and use the PID controller
        to generate an output. Then, map that output to differential thrust.
        """
        # Only run if a desired yaw is set.
        if self.desired_yaw is None:
            return

        # Compute the heading error (normalized to [-pi, pi])
        error = self.normalize_angle(self.desired_yaw - self.current_yaw)
        self.get_logger().debug(f"Yaw error: {math.degrees(error):.1f}°")

        # If the error is within tolerance, stop the motors.
        if abs(error) < self.yaw_tolerance:
            self.send_thrusts(0.0, 0.0)
            self.get_logger().info("Desired yaw achieved. Motors stopped.")
            return

        # Compute PID output (this is the differential command)
        pid_output = self.angular_pid.compute(error)
        # Multiply by the thrust_scale to generate a higher differential thrust command.
        angular_cmd = self.thrust_scale * pid_output

        # Map the angular command to differential thrust.
        # For rotation only (no forward thrust):
        #   left_thrust = -angular_cmd, right_thrust = angular_cmd.
        left_thrust = -angular_cmd
        right_thrust = angular_cmd

        # Optionally, saturate the thrust commands to a maximum (e.g., 100%).
        max_thrust = 100.0
        left_thrust = max(min(left_thrust, max_thrust), -max_thrust)
        right_thrust = max(min(right_thrust, max_thrust), -max_thrust)

        self.send_thrusts(left_thrust, right_thrust)
        self.get_logger().info(
            f"Curret: yaw = {math.degrees(self.current_yaw):.1f}°, Rotating: error = {math.degrees(error):.1f}°, PID output = {pid_output:.2f}, scaled command: left = {left_thrust:.1f}%, right = {right_thrust:.1f}%"
        )

    def send_thrusts(self, left: float, right: float):
        """
        Publish the differential thrust commands.
        A positive command on the starboard motor and a negative command on the port motor
        should produce a rotation (adjust the sign if needed for your motor configuration).
        """
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = left
        right_msg.data = right
        self.port_motor_pub.publish(right_msg)
        self.stbd_motor_pub.publish(left_msg)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize an angle to the range [-pi, pi].
        """
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = SetYawController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        node.get_logger().info("SetYawController node is running.")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("SetYawController node is shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
