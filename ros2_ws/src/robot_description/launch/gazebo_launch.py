from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/home/blueboat_slam/ros2_ws/src/robot_description/worlds/my_world.world', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py',
             arguments=['-entity', 'my_robot', '-file', '/home/blueboat_slam/ros2_ws/src/robot_description/urdf/robot.urdf'],
             output='screen'),
    ])
