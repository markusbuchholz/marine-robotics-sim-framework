# ros2 topic pub --once /bluerov2/waypoint std_msgs/msg/Float32MultiArray "{data: [1.0, 0.0, -2.0, 2.0, 2.0, -2.0, 3.0, -2.0, -4.0]}"
# ros2 topic pub --once /bluerov2/waypoint std_msgs/msg/Float32MultiArray "{data: [0.0, 0.0, -2.0]}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from pymavlink import mavutil
from time import sleep
import math

class MavlinkROS2Controller(Node):

    def __init__(self):
        super().__init__('mavlink_controller')
        self.vehicle = self.create_connection()
        self.vehicle.wait_heartbeat()
        self.get_logger().info("Heartbeat received")

        self.default_waypoints = [
            {'x': 1.0, 'y': 0.0, 'z': -2.0},
            {'x': 2.0, 'y': 2.0, 'z': -2.0},
            {'x': 3.0, 'y': -2.0, 'z': -4.0},
            {'x': 4.0, 'y': 0.0, 'z': -4.0},
            {'x': 5.0, 'y': 2.0, 'z': -6.0}
        ]
        self.waypoints = []
        self.current_waypoint = None
        self.mission_active = False
        self.use_default_waypoints = False  # Flag to select waypoint source

        self.create_subscription(Float32MultiArray, '/bluerov2/waypoint', self.waypoint_callback, 10)
        self.timer = self.create_timer(1.0, self.execute_mission_callback)

        self.set_guided_mode_hold_position_local()
        self.arm_vehicle()

    def create_connection(self):
        return mavutil.mavlink_connection('udpin:0.0.0.0:14550')

    def waypoint_callback(self, msg):
        points = msg.data
        if len(points) % 3 != 0:
            self.get_logger().warn("Received invalid waypoint array length!")
            return

        self.waypoints.clear()
        for i in range(0, len(points), 3):
            self.waypoints.append({'x': points[i], 'y': points[i+1], 'z': points[i+2]})
        self.get_logger().info(f"Waypoints updated from topic: {self.waypoints}")

    def set_vehicle_mode(self, mode):
        mode_id = self.vehicle.mode_mapping()[mode]
        self.vehicle.set_mode(mode_id)
        self.get_logger().info(f"Vehicle mode set to {mode}")

    def arm_vehicle(self):
        self.vehicle.arducopter_arm()
        self.vehicle.motors_armed_wait()
        self.get_logger().info("Vehicle armed")

    def disarm_vehicle(self):
        self.vehicle.arducopter_disarm()
        self.vehicle.motors_disarmed_wait()
        self.get_logger().info("Vehicle disarmed")

    def fetch_current_state_local(self):
        default_position = {'x': 0, 'y': 0, 'z': 0}
        default_orientation = {'yaw': 0}

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            0, 0, 0, 0, 0, 0, 0
        )
        position_message = self.vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
        position = default_position if not position_message else {
            'x': position_message.x,
            'y': position_message.y,
            'z': position_message.z
        }

        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            0, 0, 0, 0, 0, 0, 0
        )
        attitude_message = self.vehicle.recv_match(type='ATTITUDE', blocking=True, timeout=5)
        orientation = default_orientation if not attitude_message else {'yaw': attitude_message.yaw}

        return {'x': position['x'], 'y': position['y'], 'z': -position['z'], 'yaw': orientation['yaw']}

    def wait_for_valid_position(self, max_attempts=30):
        for attempt in range(max_attempts):
            current_state = self.fetch_current_state_local()
            if current_state['x'] != 0 or current_state['y'] != 0 or current_state['z'] != 0:
                return current_state
            self.get_logger().warn(f"Invalid position data, retrying... ({attempt + 1}/{max_attempts})")
            sleep(1)
        raise TimeoutError("Failed to get a valid position")

    def set_guided_mode_hold_position_local(self):
        self.get_logger().info("Setting GUIDED mode and holding position...")
        current_state = self.wait_for_valid_position()

        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4
        )

        self.vehicle.mav.set_position_target_local_ned_send(
            0,
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            current_state['x'], current_state['y'], current_state['z'],
            0, 0, 0, 0, 0, 0,
            current_state['yaw'], 0
        )
        return current_state

    def send_position_request(self, x, y, z):
        type_mask = 0b0000111111111000
        coordinate_frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

        self.vehicle.mav.set_position_target_local_ned_send(
            0, self.vehicle.target_system, self.vehicle.target_component,
            coordinate_frame, type_mask, x, y, z, 0, 0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info(f"Position request sent: x={x}, y={y}, z={z}")

    def has_reached_position(self, current, target, threshold=1.0):
        distance = math.sqrt((current['x'] - target['x'])**2 + (current['y'] - target['y'])**2 + (current['z'] - target['z'])**2)
        return distance < threshold

    def execute_mission_callback(self):
        if not self.mission_active:
            waypoint_list = self.default_waypoints if self.use_default_waypoints else self.waypoints
            if waypoint_list:
                self.current_waypoint = waypoint_list.pop(0)
                self.send_position_request(
                    self.current_waypoint['x'],
                    self.current_waypoint['y'],
                    -self.current_waypoint['z']  # Important: Negate z when sending!
                )
                self.mission_active = True
                self.get_logger().info(f"Moving to waypoint: {self.current_waypoint}")

        elif self.mission_active:
            current_pos = self.fetch_current_state_local()
            self.get_logger().info(f"Current Position: {current_pos}, Target: {self.current_waypoint}")
            if self.has_reached_position(current_pos, self.current_waypoint, threshold=1.5):
                self.get_logger().info(f"Reached waypoint: {self.current_waypoint}")
                self.mission_active = False



def main(args=None):
    rclpy.init(args=args)
    controller = MavlinkROS2Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()