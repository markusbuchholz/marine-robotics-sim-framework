#!/usr/bin/env python3
# Markus Buchholz, 2025
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Int32MultiArray, Int32

from tf2_ros import TransformBroadcaster

from pymavlink import mavutil
import threading
import time
import math
import numpy as np
import tf_transformations


class BlueROV2ROSInterface(Node):
    def __init__(self):
        super().__init__('bluerov2_ros_interface')
        
        self.passthrough_enabled = False

        # Declare Parameters
        self.declare_parameter('mavlink_connection', 'udpin:0.0.0.0:14550')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('data_stream_rate', 4)  # Hz for data stream
        self.declare_parameter('max_attempts', 200)
        self.declare_parameter('margin', 0.2)
        self.declare_parameter('timeout', 60)

        # **New Parameters: Per-Axis Gain Factors**
        self.declare_parameter('gain_forward', -1.0)  # Surge
        self.declare_parameter('gain_lateral', 1.0)   # Sway
        self.declare_parameter('gain_heave', 1.0)     # Heave
        self.declare_parameter('gain_roll', 1.0)      # Roll
        self.declare_parameter('gain_pitch', 1.0)     # Pitch
        self.declare_parameter('gain_yaw', 1.0)       # Yaw

        # Get Parameters
        mavlink_connection_str = self.get_parameter('mavlink_connection').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        data_stream_rate = self.get_parameter('data_stream_rate').get_parameter_value().integer_value
        self.max_attempts = self.get_parameter('max_attempts').get_parameter_value().integer_value
        self.margin = self.get_parameter('margin').get_parameter_value().double_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().integer_value

        # **Retrieve Per-Axis Gains**
        self.gain_forward = self.get_parameter('gain_forward').get_parameter_value().double_value
        self.gain_lateral = self.get_parameter('gain_lateral').get_parameter_value().double_value
        self.gain_heave = self.get_parameter('gain_heave').get_parameter_value().double_value
        self.gain_roll = self.get_parameter('gain_roll').get_parameter_value().double_value
        self.gain_pitch = self.get_parameter('gain_pitch').get_parameter_value().double_value
        self.gain_yaw = self.get_parameter('gain_yaw').get_parameter_value().double_value

        # **Log the Gains**
        self.get_logger().info('Control Gains:')
        self.get_logger().info(f'  Forward (Surge): {self.gain_forward}')
        self.get_logger().info(f'  Lateral (Sway): {self.gain_lateral}')
        self.get_logger().info(f'  Heave: {self.gain_heave}')
        self.get_logger().info(f'  Roll: {self.gain_roll}')
        self.get_logger().info(f'  Pitch: {self.gain_pitch}')
        self.get_logger().info(f'  Yaw: {self.gain_yaw}')

        # Define Thruster Ranges and Input Ranges
        self.thrusterRanges = [1100.0, 1900.0]  # PWM range for thrusters
        self.thrusterInputRanges = [-3.0, 3.0]   # Control input range

        # Initialize MAVLink connection
        self.conn = mavutil.mavlink_connection(mavlink_connection_str)
        self.get_logger().info(f'Connecting to ArduPilot via {mavlink_connection_str}...')
        self.conn.wait_heartbeat()
        self.get_logger().info(f'Heartbeat from system (system {self.conn.target_system} component {self.conn.target_component})')

        self.num_thrusters = 8  # Adjust based on your ROV's thrusters

        self.backup_params = self.backup_thruster_params()

        self.enable_passthrough_mode()

        self.set_stabilize_mode_and_arm()

        self.request_mavlink_messages()

        self.data_lock = threading.Lock()
        self.data = {}

        self.A = np.array([
            [0.7071,  0.7071, -0.7071, -0.7071, 0,       0,       0,       0],
            [-0.7071, 0.7071, -0.7071,  0.7071, 0,       0,       0,       0],
            [0,       0,       0,       0,      -1.0000, -1.0000, -1.0000, -1.0000],
            [0,       0,       0,       0,       0.2180,  0.2180, -0.2180, -0.2180],
            [0,       0,       0,       0,       0.1200, -0.1200,  0.1200, -0.1200],
            [-0.1888, 0.1888,  0.1888, -0.1888, 0,       0,       0,       0]
        ])

        # Set Up ROS 2 Publishers and Subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        qos_profile_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.custom_qos = qos_profile

        self.custom_qos_thrust = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.odometry_publisher = self.create_publisher(
            Odometry,
            '/bluerov2/odometry',
            qos_profile_reliable,
            callback_group=ReentrantCallbackGroup()
        )

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/bluerov2/cmd_vel',
            self._cmd_vel_callback,
            qos_profile,
            callback_group=ReentrantCallbackGroup()
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.servo_output_publisher = self.create_publisher(
            Int32MultiArray,
            '/bluerov2/servo_outputs',
            qos_profile_reliable,
            callback_group=ReentrantCallbackGroup()
        )

        

        # **Fixed Subscription Topics to Match Published Topics**
        for i in range(1, self.num_thrusters + 1):
            topic_name = f'/bluerov2/thruster_{i}_pwm_set'  # Reverted back to '_pwm_set'
            self.create_subscription(
                Int32,
                topic_name,
                self.create_thruster_callback(i),
                self.custom_qos_thrust,
                callback_group=ReentrantCallbackGroup()
            )
            self.get_logger().info(f'Subscribed to {topic_name}')

        self.mavlink_thread = threading.Thread(target=self.mavlink_listener, daemon=True)
        self.mavlink_thread.start()

        self.timer = self.create_timer(1.0 / 30.0, self._publish_odometry_and_servo_outputs)

        self.get_logger().info('BlueROV2 ROS Interface node initialized.')

    def send_rc_override_for_pwm_set(self, channels):
        self.get_logger().debug('Sending PWM to thrusters.')
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,
            self.conn.target_component,
            channels[0], channels[1], channels[2], channels[3],
            channels[4], channels[5], channels[6], channels[7]
        )

    def control_motor(self, motor_number, pwm_value):
        channels = [1500] * 8
        channels[motor_number - 1] = pwm_value
        self.send_rc_override_for_pwm_set(channels)

    def create_thruster_callback(self, thruster_num):
        def callback(msg):
            self.get_logger().info(f'Received PWM command for Thruster {thruster_num}: {msg.data}')
            if self.passthrough_enabled:
                self.control_motor(thruster_num, msg.data)
                self.get_logger().info(f'Control signal sent to Thruster {thruster_num}: PWM {msg.data}')
            else:
                self.get_logger().warn(f"Passthrough mode not enabled. Cannot control thruster {thruster_num}.")
        return callback

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
        self.get_logger().info('Params BACKUP')
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
        self.passthrough_enabled = True
        self.get_logger().info('Passthrough mode enabled.')

    def disable_passthrough_mode(self):
        """Disable passthrough mode and restore original thruster functions."""
        self.get_logger().info('Disabling passthrough mode and restoring thruster functions...')
        for i in range(1, self.num_thrusters + 1):
            original_function = self.backup_params[i - 1]
            self.set_servo_function(i, original_function)
        self.passthrough_enabled = False
        self.get_logger().info('Passthrough mode disabled and thruster functions restored.')

    def set_stabilize_mode_and_arm(self):
        """Set the vehicle's mode to STABILIZE and arm it."""
        self.get_logger().info('Setting STABILIZE mode...')
        self.conn.mav.set_mode_send(
            self.conn.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            0  # 0 typically corresponds to STABILIZE mode
        )
        time.sleep(1)  # Wait for mode to be set

        self.get_logger().info('Arming the vehicle...')
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0  # Arm
        )
        # Wait until vehicle is armed
        armed = False
        for _ in range(10):
            message = self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if message and (message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                armed = True
                self.get_logger().info('Vehicle armed successfully.')
                break
            time.sleep(0.5)
        if not armed:
            self.get_logger().error('Failed to arm the vehicle.')
            raise Exception('Vehicle failed to arm.')

    def request_mavlink_messages(self):
        """Request specific MAVLink messages like LOCAL_POSITION_NED and ATTITUDE."""
        self.get_logger().info('Requesting MAVLink messages: LOCAL_POSITION_NED and ATTITUDE...')
        # Request LOCAL_POSITION_NED
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            0, 0, 0, 0, 0, 0, 0
        )
        # Request ATTITUDE
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            0, 0, 0, 0, 0, 0, 0
        )
        # **Request SERVO_OUTPUT_RAW Messages**
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
            0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info('MAVLink message requests sent.')

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

    def _publish_odometry_and_servo_outputs(self):
        """Create and publish the odometry and servo output messages."""
        with self.data_lock:
            data = self.data.copy()

        if not data:
            self.get_logger().warn('No data received yet.')
            return

        # Publish Odometry
        self._publish_odometry(data)

        # Publish Servo Outputs
        self._publish_servo_outputs(data)

    def _publish_odometry(self, data):
        """Create and publish the odometry message based on received MAVLink data."""
        # Ensure both LOCAL_POSITION_NED and ATTITUDE data are available
        local_position_ned = data.get('LOCAL_POSITION_NED', None)
        attitude = data.get('ATTITUDE', None)

        if not local_position_ned or not attitude:
            self.get_logger().warn('Waiting for both LOCAL_POSITION_NED and ATTITUDE messages.')
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id

        # Populate position
        msg.pose.pose.position.x = local_position_ned['x']
        msg.pose.pose.position.y = -local_position_ned['y']  # Inverting Y to match ROS coordinate frame
        msg.pose.pose.position.z = -local_position_ned['z']  # Inverting Z to match ROS coordinate frame

        # Populate linear velocity
        msg.twist.twist.linear.x = -local_position_ned['vx']
        msg.twist.twist.linear.y = local_position_ned['vy']
        msg.twist.twist.linear.z = -local_position_ned['vz']

        # Populate orientation from ATTITUDE
        roll = attitude['roll']
        pitch = -attitude['pitch']
        yaw = -attitude['yaw']

        # Convert Euler angles to quaternion
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Populate angular velocity
        msg.twist.twist.angular.x = attitude['rollspeed']
        msg.twist.twist.angular.y = -attitude['pitchspeed']
        msg.twist.twist.angular.z = -attitude['yawspeed']

        # Covariance (optional: populate if EKF_STATUS_REPORT is available)
        ekf_status = data.get('EKF_STATUS_REPORT', None)
        if ekf_status:
            msg.pose.covariance[0] = ekf_status.get('pos_horiz_variance', 0.0)
            msg.pose.covariance[7] = ekf_status.get('pos_horiz_variance', 0.0)
            msg.pose.covariance[14] = ekf_status.get('pos_vert_variance', 0.0)
            msg.pose.covariance[35] = ekf_status.get('compass_variance', 0.0)
            msg.twist.covariance[0] = ekf_status.get('velocity_variance', 0.0)
            msg.twist.covariance[7] = ekf_status.get('velocity_variance', 0.0)
        else:
            # If EKF_STATUS_REPORT is not available, set default covariances or leave as zeros
            pass

        self.odometry_publisher.publish(msg)
        self.get_logger().debug('Published Odometry message.')

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug('Broadcasted TF transform.')

    def _publish_servo_outputs(self, data):
        """Extract SERVO_OUTPUT_RAW data and publish PWM values."""
        servo_output = data.get('SERVO_OUTPUT_RAW', None)
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

    def set_rc_channels_pwm(self, vals):
        """Override RC channels with provided PWM values."""
        # Ensure exactly 8 PWM values
        rc_channel_values = [int(val) for val in vals[:8]]
        while len(rc_channel_values) < 8:
            rc_channel_values.append(0)

        self.get_logger().debug(f'Sending RC Channels Override: {rc_channel_values}')

        try:
            self.conn.mav.rc_channels_override_send(
                self.conn.target_system,      # target_system
                self.conn.target_component,   # target_component
                rc_channel_values[0],
                rc_channel_values[1],
                rc_channel_values[2],
                rc_channel_values[3],
                rc_channel_values[4],
                rc_channel_values[5],
                rc_channel_values[6],
                rc_channel_values[7]
            )
            self.get_logger().info(f'RC Channels Override sent: {rc_channel_values}')
        except Exception as e:
            self.get_logger().error(f'Failed to send RC override: {e}')

    def mapRanges(self, value):
        """Map a value from thruster input range to PWM range and clip."""
        # Ensure input ranges are valid
        in_min, in_max = self.thrusterInputRanges
        out_min, out_max = self.thrusterRanges

        # Perform linear mapping
        mapped_value = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        # Clip the mapped value to thrusterRanges
        clipped_value = np.clip(mapped_value, out_min, out_max)

        return clipped_value

    def _cmd_vel_callback(self, msg):
        """Handle incoming velocity commands and translate to RC overrides."""
        # Extract control inputs from cmd_vel
        # u = [forward, lateral, heave, roll, pitch, yaw]
        u_forward = msg.linear.x
        u_lateral = msg.linear.y
        u_heave = msg.linear.z
        u_roll = msg.angular.x
        u_pitch = msg.angular.y
        u_yaw = msg.angular.z

        # Assemble control input vector
        u = np.array([u_forward, u_lateral, u_heave, u_roll, u_pitch, u_yaw])

        self.get_logger().debug(f'Received cmd_vel: {msg}')
        self.get_logger().debug(f'Original Control Inputs (u): {u}')

        # **Apply Per-Axis Gains to Control Inputs**
        u_scaled = np.array([
            self.gain_forward * u_forward,  # Surge
            self.gain_lateral * u_lateral,  # Sway
            self.gain_heave * u_heave,      # Heave
            self.gain_roll * u_roll,        # Roll
            self.gain_pitch * u_pitch,      # Pitch
            self.gain_yaw * u_yaw           # Yaw
        ])

        self.get_logger().debug(f'Scaled Control Inputs (u_scaled): {u_scaled}')

        # Compute thruster commands (t = A.T * u_scaled)
        # Since A is 6x8 and u_scaled is 6x1, t will be 8x1
        t = self.A.T.dot(u_scaled)

        self.get_logger().debug(f'Computed Thruster Commands (t): {t}')

        # Map thruster commands to PWM values
        pwm_thrusters = []
        for i in range(self.num_thrusters):
            pwm = self.mapRanges(t[i])
            pwm_thrusters.append(pwm)

        self.get_logger().debug(f'Mapped PWM Thrusters: {pwm_thrusters}')

        # Send PWM overrides
        self.set_rc_channels_pwm(pwm_thrusters)

    def shutdown(self):
        """Safely shutdown the node, disarm the vehicle, and disable passthrough."""
        self.get_logger().info('Shutting down BlueROV2 ROS Interface node...')
        try:
            # Disarm the vehicle
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 0, 0, 0, 0, 0, 0  # Disarm
            )
            self.get_logger().info('Vehicle disarmed.')

            # Disable passthrough mode
            self.disable_passthrough_mode()

            # Close MAVLink connection
            self.conn.close()
            self.get_logger().info('MAVLink connection closed.')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BlueROV2ROSInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt detected. Initiating shutdown...')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
