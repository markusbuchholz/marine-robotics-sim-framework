#!/usr/bin/env python3
# Markus Buchholz, 2025 (Modified for Blueboat)
# This program implements both the Blueboat ROS Interface and also launches
# the official joy node (ros2 run joy joy_node) via a subprocess.
# It maps joystick inputs to arm/disarm and motor thrust commands.
#
# Added publishers for additional topics:
#   - /compass: Publishes compass heading (from VFR_HUD messages) as Float32.
#   - /imu: Publishes IMU data (from ATTITUDE and RAW_IMU messages) as sensor_msgs/Imu.
#   - /blueboat_roll, /blueboat_pitch, /blueboat_yaw: Publish the normalized
#     Euler angles (in degrees) for roll, pitch, and yaw, respectively.

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Int32MultiArray, Float32
from sensor_msgs.msg import Joy, Imu
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3

from pymavlink import mavutil
import threading
import time
import numpy as np
import subprocess  # Used to launch the external joy node
import math

########################################################################
# Blueboat ROS Interface Node
########################################################################
class BlueboatROSInterface(Node):
    def __init__(self):
        super().__init__('blueboat_ros_interface')

        # Declare parameters
        self.declare_parameter('mavlink_connection', 'udpin:0.0.0.0:14560')
        self.declare_parameter('data_stream_rate', 8)
        self.declare_parameter('num_thrusters', 8)
        self.declare_parameter('uuv_name', 'blueboat')  # Parameter for uuv_name

        # Retrieve parameters
        mavlink_connection_str = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        data_stream_rate = self.get_parameter('data_stream_rate').get_parameter_value().integer_value
        self.num_thrusters = self.get_parameter('num_thrusters').get_parameter_value().integer_value
        self.uuv_name = self.get_parameter('uuv_name').get_parameter_value().string_value

        self.get_logger().info(f'Connecting to ArduPilot via {mavlink_connection_str}...')
        self.conn = mavutil.mavlink_connection(mavlink_connection_str)
        self.conn.wait_heartbeat()
        self.get_logger().info(f'Heartbeat from system (system {self.conn.target_system} component {self.conn.target_component})')

        # Request data stream (all data, 1 Hz update rate)
        self.conn.mav.request_data_stream_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            1, 1
        )

        # Define PWM ranges and input ranges
        self.thrusterRanges = [1100.0, 1900.0]  # PWM range for thrusters
        self.thrusterInputRanges = [-3.0, 3.0]   # Control input range

        # (Optional) Backup parameters if needed
        # self.backup_params = self.backup_thruster_params()

        # Initial mode and arm command. Uncomment below if you wish to enable passthrough.
        self.set_stabilize_mode_and_arm()
        # self.enable_passthrough_mode()  # Uncomment if passthrough mode is desired

        # Define QoS profiles
        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publisher for servo outputs
        self.servo_output_publisher = self.create_publisher(
            Int32MultiArray,
            '/blueboat/servo_outputs',
            qos_profile_reliable,
            callback_group=ReentrantCallbackGroup()
        )

        # Subscription for cmd_vel (if needed)
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/blueboat/cmd_vel',
            self._cmd_vel_callback,
            qos_profile_best_effort,
            callback_group=ReentrantCallbackGroup()
        )

        # Define custom QoS for motor thrust topics
        self.custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions for motor thrust commands (will be published by the joystick node)
        self.motor_port_subscription = self.create_subscription(
            Float32,
            f'{self.uuv_name}/send_port_motor_0_100_thrust',
            self.motor_port_thrust_callback,
            self.custom_qos,
            callback_group=ReentrantCallbackGroup()
        )
        self.motor_stbd_subscription = self.create_subscription(
            Float32,
            f'{self.uuv_name}/send_stbd_motor_0_100_thrust',
            self.motor_stbd_thrust_callback,
            self.custom_qos,
            callback_group=ReentrantCallbackGroup()
        )

        # --------------------------
        # New Publishers for /compass and /imu topics
        # --------------------------
        # Publisher for compass heading (Float32)
        self.compass_publisher = self.create_publisher(
            Float32,
            '/compass',
            qos_profile_reliable,
            callback_group=ReentrantCallbackGroup()
        )
        # Timer to publish compass heading at 1 Hz
        self.compass_timer = self.create_timer(1.0, self.publish_compass)

        # Publisher for IMU data (sensor_msgs/Imu)
        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu',
            qos_profile_reliable,
            callback_group=ReentrantCallbackGroup()
        )
        # Timer to publish IMU data at 30 Hz (adjust as needed)
        self.imu_timer = self.create_timer(1.0/30.0, self.publish_imu)
        # --------------------------

        # --------------------------
        # New Publishers for Euler angles as separate topics
        # --------------------------
        self.roll_publisher = self.create_publisher(
            Float32,
            '/blueboat_roll',
            qos_profile_reliable,
            callback_group=ReentrantCallbackGroup()
        )
        self.pitch_publisher = self.create_publisher(
            Float32,
            '/blueboat_pitch',
            qos_profile_reliable,
            callback_group=ReentrantCallbackGroup()
        )
        self.yaw_publisher = self.create_publisher(
            Float32,
            '/blueboat_yaw',
            qos_profile_reliable,
            callback_group=ReentrantCallbackGroup()
        )

        self.data_lock = threading.Lock()
        self.data = {}

        # Start the MAVLink listener thread.
        self.mavlink_thread = threading.Thread(target=self.mavlink_listener, daemon=True)
        self.mavlink_thread.start()

        # Timer to publish servo outputs at 30 Hz.
        self.timer = self.create_timer(1.0 / 30.0, self._publish_servo_outputs)

        self.get_logger().info('Blueboat ROS Interface node initialized.')

    def backup_thruster_params(self):
        """Backup the current SERVO_FUNCTION parameters."""
        backup = []
        for i in range(1, self.num_thrusters + 1):
            param_name = f'SERVO{i}_FUNCTION'
            self.conn.mav.param_request_read_send(
                self.conn.target_system,
                self.conn.target_component,
                param_name.encode('utf-8'),
                -1
            )
            message = self.conn.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if message:
                backup.append(int(message.param_value))
                self.get_logger().info(f'Backed up {param_name}: {int(message.param_value)}')
            else:
                backup.append(0)
                self.get_logger().warn(f'Failed to backup {param_name}, defaulting to 0')
        return backup

    def set_servo_function(self, servo_num, function):
        """Set the SERVO_FUNCTION parameter for a specific thruster."""
        param_name = f'SERVO{servo_num}_FUNCTION'
        self.conn.mav.param_set_send(
            self.conn.target_system,
            self.conn.target_component,
            param_name.encode('utf-8'),
            function,
            mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        self.get_logger().info(f'Set {param_name} to {function}')
        time.sleep(0.1)  # Small delay to ensure the command is processed

    def enable_passthrough_mode(self):
        """Enable passthrough mode for all thrusters."""
        self.get_logger().info('Enabling passthrough mode for thrusters...')
        for i in range(1, self.num_thrusters + 1):
            self.set_servo_function(i, 1)  # 1 typically stands for passthrough
        self.get_logger().info('Passthrough mode enabled.')

    def disable_passthrough_mode(self):
        """Disable passthrough mode and restore original thruster functions."""
        self.get_logger().info('Disabling passthrough mode and restoring thruster functions...')
        for i in range(1, self.num_thrusters + 1):
            original_function = self.backup_params[i-1]
            self.set_servo_function(i, original_function)
        self.get_logger().info('Passthrough mode disabled and thruster functions restored.')

    def set_stabilize_mode_and_arm(self):
        """Set the vehicle's mode to STABILIZE and arm it."""
        # Uncomment and modify the following to set a specific mode if needed.
        # self.get_logger().info('Setting STABILIZE mode...')
        # self.conn.mav.set_mode_send(
        #     self.conn.target_system,
        #     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        #     0
        # )
        time.sleep(1)  # Give some time for mode to be set

        self.get_logger().info('Arming the vehicle (initial command)...')
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # Arm command
        )
        armed = False
        for _ in range(10):
            message = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if message and (message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                armed = True
                self.get_logger().info('Vehicle armed successfully.')
                break
            time.sleep(0.5)
        if not armed:
            self.get_logger().error('Failed to arm the vehicle during initialization.')
            raise Exception('Vehicle failed to arm.')

    def arm_vehicle(self):
        """Arm the vehicle via joystick command."""
        self.get_logger().info('Arming the vehicle (joystick command)...')
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # Arm command
        )

    def disarm_vehicle(self):
        """Disarm the vehicle via joystick command."""
        self.get_logger().info('Disarming the vehicle (joystick command)...')
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0  # Disarm command
        )

    def mavlink_listener(self):
        """Continuously listen for MAVLink messages and update data."""
        while rclpy.ok():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=1)
                if msg:
                    msg_type = msg.get_type()
                    with self.data_lock:
                        self.data[msg_type] = msg.to_dict()
                    self.get_logger().debug(f'Received MAVLink message: {msg_type}')
            except Exception as e:
                self.get_logger().error(f'Error while receiving MAVLink message: {e}')
                time.sleep(0.1)

    def _publish_servo_outputs(self):
        """Extract SERVO_OUTPUT_RAW data and publish PWM values."""
        with self.data_lock:
            servo_output = self.data.get('SERVO_OUTPUT_RAW', None)

        if not servo_output:
            self.get_logger().warn('SERVO_OUTPUT_RAW message not received yet.')
            return

        # Extract servo1_raw to servo8_raw
        pwm_values = [
            servo_output.get('servo1_raw', 0),
            servo_output.get('servo2_raw', 0),
            servo_output.get('servo3_raw', 0),
            servo_output.get('servo4_raw', 0),
            servo_output.get('servo5_raw', 0),
            servo_output.get('servo6_raw', 0),
            servo_output.get('servo7_raw', 0),
            servo_output.get('servo8_raw', 0)
        ]

        pwm_msg = Int32MultiArray()
        pwm_msg.data = pwm_values
        self.servo_output_publisher.publish(pwm_msg)
        self.get_logger().debug(f'Published PWM Values: {pwm_values}')

    def mapRanges(self, value):
        """Map a value from thruster input range to PWM range and clip."""
        in_min, in_max = self.thrusterInputRanges
        out_min, out_max = self.thrusterRanges
        # Linear mapping
        mapped_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        # Clip the mapped value
        clipped_value = np.clip(mapped_value, out_min, out_max)
        return clipped_value

    def _cmd_vel_callback(self, msg):
        """Handle incoming velocity commands and translate to RC overrides for Blueboat."""
        override = []
        try:
            override.append(1500 - msg.linear.y)               # Example pitch adjustment
            override.append(int(self.mapRanges(msg.angular.x)))  # Mapped value for a channel
            override.append(int(self.mapRanges(msg.linear.z)))
            override.append(int(3000 - self.mapRanges(msg.angular.z)))  # Example forward
            override.append(int(self.mapRanges(msg.linear.x)))
            override.append(65535)  # Lateral (no change)
            override.append(65535)  # Light strength (no change)
            override.append(65535)  # Camera servo tilt (no change)

            while len(override) < 8:
                override.append(65535)

            self.get_logger().debug(f'Sending RC Channels Override: {override}')
            self.set_rc_channels_pwm(override)
            self.get_logger().info(f'RC Channels Override sent: {override}')
        except Exception as e:
            self.get_logger().error(f'Failed to process cmd_vel: {e}')

    def set_rc_channels_pwm(self, vals):
        """Override RC channels with provided PWM values."""
        rc_channel_values = [int(val) for val in vals[:8]]
        while len(rc_channel_values) < 8:
            rc_channel_values.append(65535)  # 65535 typically means no change

        self.get_logger().debug(f'Sending RC Channels Override: {rc_channel_values}')
        try:
            self.conn.mav.rc_channels_override_send(
                self.conn.target_system,      # target_system
                self.conn.target_component,   # target_component
                *rc_channel_values
            )
            self.get_logger().info(f'RC Channels Override sent: {rc_channel_values}')
        except Exception as e:
            self.get_logger().error(f'Failed to send RC override: {e}')

    # Callback for Port Motor Thrust
    def motor_port_thrust_callback(self, msg):
        """Handle incoming port motor thrust commands."""
        thrust_value = int(msg.data)
        self.get_logger().info(f'Received port motor thrust: {thrust_value}')
        self.send_motor_thrust(3, thrust_value)  # Assuming motor number 3 for port

    # Callback for Starboard Motor Thrust
    def motor_stbd_thrust_callback(self, msg):
        """Handle incoming starboard motor thrust commands."""
        thrust_value = int(msg.data)
        self.get_logger().info(f'Received starboard motor thrust: {thrust_value}')
        self.send_motor_thrust(4, thrust_value)  # Assuming motor number 4 for starboard

    def send_motor_thrust(self, motor_number, thrust_value):
        """
        Sends a MAV_CMD_DO_MOTOR_TEST command to the vehicle.
        :param motor_number: Motor number (1-8 for an 8-thruster vehicle)
        :param thrust_value: Thrust value (0-100 for percent)
        """
        thrust_type = 0  # 0 = Percent
        duration = 1.0  # Duration per cycle (based on 30 Hz)
        self.get_logger().info(f'Sending MOTOR_TEST command: Motor {motor_number}, Thrust {thrust_value}%')
        try:
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                0,
                motor_number,
                thrust_type,
                thrust_value,
                duration,
                0, 0, 0
            )
            self.get_logger().info(f'MOTOR_TEST command sent to motor {motor_number} with thrust {thrust_value}%')
        except Exception as e:
            self.get_logger().error(f'Failed to send MOTOR_TEST command: {e}')

    def publish_compass(self):
        """Publish compass heading extracted from VFR_HUD messages to /compass."""
        with self.data_lock:
            vfr_hud = self.data.get('VFR_HUD', None)
        if vfr_hud is None:
            self.get_logger().warn("VFR_HUD data not available for compass heading.")
            return
        heading = vfr_hud.get('heading', None)
        if heading is None:
            self.get_logger().warn("Heading value not found in VFR_HUD.")
            return
        # Normalize heading to 0-360 degrees
        heading_normalized = heading % 360
        msg = Float32()
        msg.data = float(heading_normalized)
        self.compass_publisher.publish(msg)
        self.get_logger().debug(f"Published compass heading: {heading_normalized} degrees")

    def publish_imu(self):
        """Publish IMU data constructed from ATTITUDE and RAW_IMU messages to /imu,
        and publish the normalized Euler angles to separate topics."""
        with self.data_lock:
            attitude = self.data.get('ATTITUDE', None)
            raw_imu = self.data.get('RAW_IMU', None)
        if attitude is None or raw_imu is None:
            self.get_logger().warn("Incomplete IMU data: waiting for both ATTITUDE and RAW_IMU messages.")
            return

        # Extract Euler angles (in radians) from the ATTITUDE message
        roll = float(attitude.get('roll', 0.0))
        pitch = float(attitude.get('pitch', 0.0))
        yaw = float(attitude.get('yaw', 0.0))
        # Convert to degrees and normalize to [0, 360)
        roll_deg = math.degrees(roll) % 360
        pitch_deg = math.degrees(pitch) % 360
        yaw_deg = math.degrees(yaw) % 360

        # Publish the Euler angles on separate topics
        roll_msg = Float32()
        roll_msg.data = roll_deg
        self.roll_publisher.publish(roll_msg)
        
        pitch_msg = Float32()
        pitch_msg.data = pitch_deg
        self.pitch_publisher.publish(pitch_msg)
        
        yaw_msg = Float32()
        yaw_msg.data = yaw_deg
        self.yaw_publisher.publish(yaw_msg)

        # Convert the original (radian) values to a quaternion for the IMU message
        q = self.euler_to_quaternion(roll, pitch, yaw)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.orientation.x = float(q[0])
        imu_msg.orientation.y = float(q[1])
        imu_msg.orientation.z = float(q[2])
        imu_msg.orientation.w = float(q[3])

        # Angular velocity from ATTITUDE
        imu_msg.angular_velocity.x = float(attitude.get('rollspeed', 0.0))
        imu_msg.angular_velocity.y = float(attitude.get('pitchspeed', 0.0))
        imu_msg.angular_velocity.z = float(attitude.get('yawspeed', 0.0))

        # Linear acceleration from RAW_IMU
        imu_msg.linear_acceleration.x = float(raw_imu.get('xacc', 0.0))
        imu_msg.linear_acceleration.y = float(raw_imu.get('yacc', 0.0))
        imu_msg.linear_acceleration.z = float(raw_imu.get('zacc', 0.0))

        # (Optional) Set covariance arrays as needed
        imu_msg.orientation_covariance = [0.0] * 9
        imu_msg.angular_velocity_covariance = [0.0] * 9
        imu_msg.linear_acceleration_covariance = [0.0] * 9

        self.imu_publisher.publish(imu_msg)
        self.get_logger().debug("Published IMU data.")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (in radians) to quaternion."""
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
             math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
             math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
             math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def shutdown(self):
        """Safely shutdown the node, disarm the vehicle, and disable passthrough."""
        self.get_logger().info('Shutting down Blueboat ROS Interface node...')
        try:
            # Disarm the vehicle
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0  # Disarm command
            )
            self.get_logger().info('Vehicle disarmed.')
            # Close MAVLink connection
            self.conn.close()
            self.get_logger().info('MAVLink connection closed.')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')

########################################################################
# Joystick Node
########################################################################
class JoystickNode(Node):
    def __init__(self, blueboat_interface):
        super().__init__('joystick')
        self.blueboat = blueboat_interface
        
        cmd_vel_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.cmd_vel_publisher = self.create_publisher(Twist, "/blueboat/cmd_vel", qos_profile=cmd_vel_qos)

        # Subscribe to the /joy topic (published by the external joy_node)
        self.joy_subscription = self.create_subscription(
            Joy,
             '/joy',
            #'/joy_fake',
            #self.joy_callback_differential,
            self.joy_callaback_pwm,
            qos_profile_sensor_data
        )
        self.get_logger().info("Joystick node initialized. Listening to /joy.")

        # Publishers to send motor thrust commands.
        self.port_motor_pub = self.create_publisher(
            Float32,
            f'{self.blueboat.uuv_name}/send_port_motor_0_100_thrust',
            10
        )
        self.stbd_motor_pub = self.create_publisher(
            Float32,
            f'{self.blueboat.uuv_name}/send_stbd_motor_0_100_thrust',
            10
        )

    def direct_joy_callback(self, msg):
        """Process joystick messages and trigger commands/motor thrust."""
        # Print the entire Joy message for debugging
        self.get_logger().info(f"Joy message received: buttons: {msg.buttons}, axes: {msg.axes}")

        # Check button indices (buttons are zero-indexed)
        if len(msg.buttons) >= 8:
            if msg.buttons[7] == 1:
                self.get_logger().info("Button 7 pressed: Arming vehicle")
                self.blueboat.arm_vehicle()
            if msg.buttons[6] == 1:
                self.get_logger().info("Button 6 pressed: Disarming vehicle")
                self.blueboat.disarm_vehicle()
        else:
            self.get_logger().warn("Received Joy message does not have at least 8 buttons!")

        # Map axis 4 to port motor thrust.
        if len(msg.axes) > 4:
            # Map from [-1.0, 1.0] to [-100, 100] then apply a scaling factor of 0.5 -> [-50, 50]
            port_val = float(msg.axes[4] * 100 * 0.5)
            port_msg = Float32()
            port_msg.data = port_val
            self.port_motor_pub.publish(port_msg)
            self.get_logger().info(f"Published port motor thrust: {port_val}")

        # Map axis 1 to starboard motor thrust.
        if len(msg.axes) > 1:
            stbd_val = float(msg.axes[1] * 100 * 0.5)
            stbd_msg = Float32()
            stbd_msg.data = stbd_val
            self.stbd_motor_pub.publish(stbd_msg)
            self.get_logger().info(f"Published starboard motor thrust: {stbd_val}")
            
            

    def joy_callaback_pwm (self, msg):
        
        linear = Vector3()
        angular = Vector3()
        
        if len(msg.buttons) >= 8:
            if msg.buttons[7] == 1:
                self.get_logger().info("Button 7 pressed: Arming vehicle")
                self.blueboat.arm_vehicle()
            if msg.buttons[6] == 1:
                self.get_logger().info("Button 6 pressed: Disarming vehicle")
                self.blueboat.disarm_vehicle()
        else:
            self.get_logger().warn("Received Joy message does not have at least 8 buttons!")
        
        value_y = float(msg.axes[0])      # forward/backward axis, expected [-1, 1]
        value_z = float(msg.axes[4]) 
        linear.y = value_y * 100.0
        linear.z = value_z * 3.0
        
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear = linear
        cmd_vel_msg.angular = angular
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        
        

        
            
    def joy_callback_differential(self, msg):
        """
        Process joystick messages to control differential drive with two modes:
        1. Differential trimming: When forward axis is active, both motors move forward
            with one motor’s speed reduced based on the differential axis.
        2. Pivot-in-place: When forward axis is neutral, the differential axis commands
            opposite speeds for the motors so the robot spins in place.
        """
        # Log the received joystick data.
        self.get_logger().info(f"Joy message received: buttons: {msg.buttons}, axes: {msg.axes}")

        # Process buttons for arm/disarm commands.
        if len(msg.buttons) >= 8:
            if msg.buttons[7] == 1:
                self.get_logger().info("Button 7 pressed: Arming vehicle")
                self.blueboat.arm_vehicle()
            if msg.buttons[6] == 1:
                self.get_logger().info("Button 6 pressed: Disarming vehicle")
                self.blueboat.disarm_vehicle()
        else:
            self.get_logger().warn("Received Joy message does not have at least 8 buttons!")

        # Ensure there are enough axes.
        if len(msg.axes) > 4:
            # Retrieve joystick inputs.
            forward_input = float(msg.axes[4])      # forward/backward axis, expected [-1, 1]
            diff_input = float(msg.axes[0])           # differential axis, expected [-1, 1]

            # Define parameters.
            max_thrust = 100.0         # full power value for each motor.
            trim_scale = 5.0           # scaling factor for differential trimming.
            deadband = 0.1             # threshold for forward axis activation.

            # Two modes: forward (or backward) drive versus pivot-in-place.
            if abs(forward_input) > deadband:
                # Mode 1: Differential trimming while moving forward/backward.
                baseline = forward_input * max_thrust
                if diff_input > 0:
                    # Positive differential: reduce port (left) motor.
                    port_thrust = baseline * (1 - trim_scale * abs(diff_input))
                    stbd_thrust = baseline
                elif diff_input < 0:
                    # Negative differential: reduce starboard (right) motor.
                    port_thrust = baseline
                    stbd_thrust = baseline * (1 - trim_scale * abs(diff_input))
                else:
                    port_thrust = baseline
                    stbd_thrust = baseline
            else:
                # Mode 2: Pivot-in-place.
                # When forward is neutral, the differential axis drives the motors in opposite directions.
                # For example, if diff_input is positive, port motor spins forward and starboard spins reverse.
                port_thrust = diff_input * max_thrust
                stbd_thrust = -diff_input * max_thrust

            # Publish motor commands.
            port_msg = Float32()
            port_msg.data = port_thrust
            self.port_motor_pub.publish(port_msg)
            self.get_logger().info(f"Published port motor thrust: {port_thrust}")

            stbd_msg = Float32()
            stbd_msg.data = stbd_thrust
            self.stbd_motor_pub.publish(stbd_msg)
            self.get_logger().info(f"Published starboard motor thrust: {stbd_thrust}")
        else:
            self.get_logger().warn("Insufficient axes data for differential control. At least 5 axes are required.")




########################################################################
# Main function
########################################################################
def main(args=None):
    rclpy.init(args=args)

    # Launch the official joy node (which reads the USB joystick) as a subprocess.
    # This is equivalent to running: ros2 run joy joy_node
    try:
        joy_proc = subprocess.Popen(['ros2', 'run', 'joy', 'joy_node'])
    except Exception as e:
        print(f"Failed to launch joy_node: {e}")
        joy_proc = None

    # Create our ROS nodes.
    blueboat_node = BlueboatROSInterface()
    joystick_node = JoystickNode(blueboat_node)

    # Use a multi-threaded executor to run nodes concurrently.
    executor = MultiThreadedExecutor()
    executor.add_node(blueboat_node)
    executor.add_node(joystick_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        blueboat_node.get_logger().info('Keyboard Interrupt detected. Initiating shutdown...')
    except Exception as e:
        blueboat_node.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Shutdown our nodes.
        blueboat_node.shutdown()
        blueboat_node.destroy_node()
        joystick_node.destroy_node()

        # Terminate the joy_node subprocess if it was launched.
        if joy_proc is not None:
            joy_proc.terminate()
            joy_proc.wait()

        rclpy.shutdown()

if __name__ == '__main__':
    main()
